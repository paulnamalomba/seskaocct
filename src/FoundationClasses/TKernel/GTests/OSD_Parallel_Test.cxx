// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>
#include <Standard_Failure.hxx>
#include <Standard_OutOfMemory.hxx>
#include <Standard_ProgramError.hxx>

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <new>
#include <random>

namespace
{

// SAXPY functor: Y[i] = scalar * X[i] + Y[i]
class SaxpyFunctor
{
public:
  SaxpyFunctor(const NCollection_Array1<double>& theX,
               NCollection_Array1<double>&       theY,
               double                            theScalar)
      : myX(theX),
        myY(theY),
        myScalar(theScalar)
  {
  }

  int Begin() const { return 0; }

  int End() const { return myX.Length(); }

  void operator()(int theIndex) const { myY(theIndex) = myScalar * myX(theIndex) + myY(theIndex); }

  void operator()(int /*theThreadIndex*/, int theIndex) const { (*this)(theIndex); }

private:
  SaxpyFunctor(const SaxpyFunctor&)      = delete;
  SaxpyFunctor& operator=(SaxpyFunctor&) = delete;

  const NCollection_Array1<double>& myX;
  NCollection_Array1<double>&       myY;
  double                            myScalar;
};

// Batched SAXPY functor for large vectors
class SaxpyBatchFunctor
{
public:
  static const int THE_BATCH_SIZE = 100000;

  SaxpyBatchFunctor(const NCollection_Array1<double>& theX,
                    NCollection_Array1<double>&       theY,
                    double                            theScalar)
      : myX(theX),
        myY(theY),
        myScalar(theScalar),
        myNbBatches((int)std::ceil((double)theX.Size() / THE_BATCH_SIZE))
  {
  }

  int Begin() const { return 0; }

  int End() const { return myNbBatches; }

  void operator()(int theBatchIndex) const
  {
    const int aLower  = theBatchIndex * THE_BATCH_SIZE;
    const int anUpper = std::min(aLower + THE_BATCH_SIZE - 1, myX.Upper());
    for (int i = aLower; i <= anUpper; ++i)
    {
      myY(i) = myScalar * myX(i) + myY(i);
    }
  }

  void operator()(int /*theThreadIndex*/, int theBatchIndex) const { (*this)(theBatchIndex); }

private:
  SaxpyBatchFunctor(const SaxpyBatchFunctor&)      = delete;
  SaxpyBatchFunctor& operator=(SaxpyBatchFunctor&) = delete;

  const NCollection_Array1<double>& myX;
  NCollection_Array1<double>&       myY;
  double                            myScalar;
  int                               myNbBatches;
};

// Matrix multiplication functor: result(i,j) = sum_k mat1(i,k)*mat2(k,j)
class MatMultFunctor
{
public:
  MatMultFunctor(const NCollection_Array2<double>& theMat1,
                 const NCollection_Array2<double>& theMat2,
                 NCollection_Array2<double>&       theResult,
                 int                               theSize)
      : myMat1(theMat1),
        myMat2(theMat2),
        myResult(theResult),
        mySize(theSize)
  {
  }

  int Begin() const { return 0; }

  int End() const { return mySize; }

  void operator()(int theIndex) const
  {
    for (int j = 0; j < mySize; ++j)
    {
      double aTmp = 0.0;
      for (int k = 0; k < mySize; ++k)
      {
        aTmp += myMat1(theIndex, k) * myMat2(k, j);
      }
      myResult(theIndex, j) = aTmp;
    }
  }

  void operator()(int /*theThreadIndex*/, int theIndex) const { (*this)(theIndex); }

private:
  MatMultFunctor(const MatMultFunctor&)      = delete;
  MatMultFunctor& operator=(MatMultFunctor&) = delete;

  const NCollection_Array2<double>& myMat1;
  const NCollection_Array2<double>& myMat2;
  NCollection_Array2<double>&       myResult;
  int                               mySize;
};

} // namespace

// Tests that OSD_Parallel::For and OSD_ThreadPool::Launcher produce the same
// result as sequential SAXPY Y[i] = scalar * X[i] + Y[i] (OCC24826).
TEST(OSD_ParallelTest, OCC24826_SaxpyParallelMatchesSequential)
{
  const int aLength = 500000;

  NCollection_Array1<double> aX(0, aLength - 1);
  NCollection_Array1<double> anYRef(0, aLength - 1);
  for (int i = 0; i < aLength; ++i)
  {
    aX(i) = anYRef(i) = static_cast<double>(i);
  }

  // Sequential reference
  {
    const SaxpyFunctor aFunctor(aX, anYRef, 1e-6);
    for (int i = 0; i < aLength; ++i)
    {
      aFunctor(i);
    }
  }

  // OSD_Parallel::For
  {
    NCollection_Array1<double> anY = aX;
    for (int i = 0; i < aLength; ++i)
    {
      anY(i) = static_cast<double>(i);
    }
    SaxpyFunctor aFunctor(aX, anY, 1e-6);
    OSD_Parallel::For(aFunctor.Begin(), aFunctor.End(), aFunctor);
    for (int i = 0; i < aLength; ++i)
    {
      EXPECT_DOUBLE_EQ(anY(i), anYRef(i)) << "Mismatch at index " << i;
    }
  }

  // OSD_ThreadPool::Launcher
  {
    NCollection_Array1<double> anY = aX;
    for (int i = 0; i < aLength; ++i)
    {
      anY(i) = static_cast<double>(i);
    }
    SaxpyFunctor             aFunctor(aX, anY, 1e-6);
    OSD_ThreadPool::Launcher aLauncher(*OSD_ThreadPool::DefaultPool());
    aLauncher.Perform(aFunctor.Begin(), aFunctor.End(), aFunctor);
    for (int i = 0; i < aLength; ++i)
    {
      EXPECT_DOUBLE_EQ(anY(i), anYRef(i)) << "Mismatch at index " << i;
    }
  }

  // OSD_Parallel::For with batched functor
  {
    NCollection_Array1<double> anY = aX;
    for (int i = 0; i < aLength; ++i)
    {
      anY(i) = static_cast<double>(i);
    }
    SaxpyBatchFunctor aFunctor(aX, anY, 1e-6);
    OSD_Parallel::For(aFunctor.Begin(), aFunctor.End(), aFunctor);
    for (int i = 0; i < aLength; ++i)
    {
      EXPECT_DOUBLE_EQ(anY(i), anYRef(i)) << "Mismatch at index " << i;
    }
  }
}

// Tests that OSD_Parallel::For and OSD_ThreadPool::Launcher produce the same
// result as sequential matrix multiplication (OCC29935).
TEST(OSD_ParallelTest, OCC29935_MatrixMultiplyParallelMatchesSequential)
{
  const int aSize = 50;

  std::mt19937               aGen(42);
  NCollection_Array2<double> aMat1(0, aSize - 1, 0, aSize - 1);
  NCollection_Array2<double> aMat2(0, aSize - 1, 0, aSize - 1);
  NCollection_Array2<double> aMatRef(0, aSize - 1, 0, aSize - 1);
  NCollection_Array2<double> aMatRes(0, aSize - 1, 0, aSize - 1);

  for (int i = 0; i < aSize; ++i)
  {
    for (int j = 0; j < aSize; ++j)
    {
      aMat1(i, j) = static_cast<double>(aGen() % 1000);
      aMat2(i, j) = static_cast<double>(aGen() % 1000);
    }
  }

  // Sequential reference
  {
    MatMultFunctor aFunctor(aMat1, aMat2, aMatRef, aSize);
    for (int i = aFunctor.Begin(); i < aFunctor.End(); ++i)
    {
      aFunctor(i);
    }
  }

  // OSD_Parallel::For
  {
    aMatRes.Init(0.0);
    MatMultFunctor aFunctor(aMat1, aMat2, aMatRes, aSize);
    OSD_Parallel::For(aFunctor.Begin(), aFunctor.End(), aFunctor);
    for (int i = 0; i < aSize; ++i)
    {
      for (int j = 0; j < aSize; ++j)
      {
        EXPECT_DOUBLE_EQ(aMatRes(i, j), aMatRef(i, j));
      }
    }
  }

  // OSD_ThreadPool::Launcher
  {
    aMatRes.Init(0.0);
    MatMultFunctor           aFunctor(aMat1, aMat2, aMatRes, aSize);
    OSD_ThreadPool::Launcher aLauncher(*OSD_ThreadPool::DefaultPool());
    aLauncher.Perform(aFunctor.Begin(), aFunctor.End(), aFunctor);
    for (int i = 0; i < aSize; ++i)
    {
      for (int j = 0; j < aSize; ++j)
      {
        EXPECT_DOUBLE_EQ(aMatRes(i, j), aMatRef(i, j));
      }
    }
  }
}

namespace
{

//! Functor raising the given kind of exception for the chosen items.
class ThrowingFunctor
{
public:
  enum Kind
  {
    Kind_OutOfMemory,
    Kind_BadAlloc,
    Kind_Failure
  };

  //! theFailingIndex < 0 makes every item fail, so that several threads fail.
  ThrowingFunctor(const Kind theKind, const int theFailingIndex)
      : myKind(theKind),
        myFailingIndex(theFailingIndex)
  {
  }

  void operator()(int theIndex) const
  {
    if (myFailingIndex >= 0 && theIndex != myFailingIndex)
    {
      return;
    }
    switch (myKind)
    {
      case Kind_OutOfMemory:
        throw Standard_OutOfMemory("injected out of memory");
      case Kind_BadAlloc:
        throw std::bad_alloc();
      case Kind_Failure:
        throw Standard_Failure("injected failure");
    }
  }

private:
  Kind myKind;
  int  myFailingIndex;
};

const int THE_THROWING_ITEMS_NB = 4096;

} // namespace

// An out-of-memory condition raised in a parallel loop keeps its type, whichever
// thread of the pool raised it and however many threads failed.
TEST(OSD_ParallelTest, OutOfMemoryInParallelLoopKeepsItsType)
{
  for (const int aFailingIndex : {0, THE_THROWING_ITEMS_NB - 1, -1})
  {
    const ThrowingFunctor aFunctor(ThrowingFunctor::Kind_OutOfMemory, aFailingIndex);
    try
    {
      OSD_Parallel::For(0, THE_THROWING_ITEMS_NB, aFunctor, false);
      ADD_FAILURE() << "No exception, failing index " << aFailingIndex;
    }
    catch (Standard_OutOfMemory const& theException)
    {
      // The other threads that failed, if any, are only counted after the text.
      const char* aText = "injected out of memory";
      EXPECT_EQ(0, strncmp(aText, theException.what(), strlen(aText))) << theException.what();
      if (aFailingIndex >= 0)
      {
        EXPECT_STREQ(aText, theException.what());
      }
    }
    catch (Standard_Failure const& theException)
    {
      ADD_FAILURE() << "Failing index " << aFailingIndex << ": " << theException.ExceptionType()
                    << ": " << theException.what();
    }
  }
}

// When several threads run out of memory, the first one is reported and the others are
// counted with the threads that failed otherwise.
TEST(OSD_ParallelTest, OutOfMemoryInSeveralThreadsCountsTheOthers)
{
  std::atomic<int> aNbThrows{0};
  try
  {
    OSD_Parallel::For(
      0,
      THE_THROWING_ITEMS_NB,
      [&aNbThrows](int) {
        ++aNbThrows;
        throw Standard_OutOfMemory("injected out of memory");
      },
      false);
    ADD_FAILURE() << "No exception";
  }
  catch (Standard_OutOfMemory const& theException)
  {
    // Each failed thread stops at its first item, so each throw is one failed thread.
    const int aNbOthers = aNbThrows.load() - 1;
    char      anExpected[256];
    if (aNbOthers == 0)
    {
      snprintf(anExpected, sizeof(anExpected), "injected out of memory");
    }
    else
    {
      snprintf(anExpected,
               sizeof(anExpected),
               "injected out of memory (and %d other failed threads)",
               aNbOthers);
    }
    EXPECT_STREQ(anExpected, theException.what());
  }
}

TEST(OSD_ParallelTest, BadAllocInParallelLoopBecomesOutOfMemory)
{
  for (const int aFailingIndex : {0, THE_THROWING_ITEMS_NB - 1, -1})
  {
    const ThrowingFunctor aFunctor(ThrowingFunctor::Kind_BadAlloc, aFailingIndex);
    EXPECT_THROW(OSD_Parallel::For(0, THE_THROWING_ITEMS_NB, aFunctor, false),
                 Standard_OutOfMemory)
      << "Failing index " << aFailingIndex;
  }
}

// Regression guard: any other failure is still reported as Standard_ProgramError.
TEST(OSD_ParallelTest, FailureInParallelLoopIsProgramError)
{
  for (const int aFailingIndex : {0, -1})
  {
    const ThrowingFunctor aFunctor(ThrowingFunctor::Kind_Failure, aFailingIndex);
    try
    {
      OSD_Parallel::For(0, THE_THROWING_ITEMS_NB, aFunctor, false);
      ADD_FAILURE() << "No exception, failing index " << aFailingIndex;
    }
    catch (Standard_OutOfMemory const&)
    {
      ADD_FAILURE() << "Standard_OutOfMemory, failing index " << aFailingIndex;
    }
    catch (Standard_ProgramError const&)
    {
    }
  }
}

// The pool is usable after a failed loop and does not report the old failure again.
TEST(OSD_ParallelTest, ParallelLoopAfterOutOfMemorySucceeds)
{
  const ThrowingFunctor aFailing(ThrowingFunctor::Kind_OutOfMemory, -1);
  EXPECT_THROW(OSD_Parallel::For(0, THE_THROWING_ITEMS_NB, aFailing, false), Standard_OutOfMemory);

  std::atomic<int> aCounter{0};
  EXPECT_NO_THROW(OSD_Parallel::For(
    0,
    THE_THROWING_ITEMS_NB,
    [&aCounter](int) { ++aCounter; },
    false));
  EXPECT_EQ(THE_THROWING_ITEMS_NB, aCounter.load());
}

// Idle threads of the pool do not report the old failure either: the next loop
// has fewer items than the pool has threads.
TEST(OSD_ParallelTest, ShortParallelLoopAfterOutOfMemorySucceeds)
{
  const ThrowingFunctor aFailing(ThrowingFunctor::Kind_OutOfMemory, -1);
  EXPECT_THROW(OSD_Parallel::For(0, THE_THROWING_ITEMS_NB, aFailing, false), Standard_OutOfMemory);

  std::atomic<int> aCounter{0};
  EXPECT_NO_THROW(OSD_Parallel::For(
    0,
    2,
    [&aCounter](int) { ++aCounter; },
    false));
  EXPECT_EQ(2, aCounter.load());
}

// Out of memory wins over the other failures of the same loop: the first item runs out
// of memory and every other item raises a plain failure.
TEST(OSD_ParallelTest, OutOfMemoryWinsOverOtherFailuresInParallelLoop)
{
  const auto aFunctor = [](int theIndex) {
    if (theIndex == 0)
    {
      throw Standard_OutOfMemory("injected out of memory");
    }
    throw Standard_Failure("injected failure");
  };

  // Each thread stops at its first item, so repeat to let the pool threads take part.
  for (int aRun = 0; aRun < 64; ++aRun)
  {
    try
    {
      OSD_Parallel::For(0, THE_THROWING_ITEMS_NB, aFunctor, false);
      ADD_FAILURE() << "No exception, run " << aRun;
    }
    catch (Standard_OutOfMemory const& theException)
    {
      // The text of the out-of-memory condition comes first; other failures are only counted.
      EXPECT_EQ(theException.what(), strstr(theException.what(), "injected out of memory"))
        << theException.what();
      EXPECT_EQ(nullptr, strstr(theException.what(), "injected failure")) << theException.what();
    }
    catch (Standard_Failure const& theException)
    {
      ADD_FAILURE() << "Run " << aRun << ": " << theException.ExceptionType() << ": "
                    << theException.what();
    }
  }
}

namespace
{

//! Gives access to the protected nested types of OSD_ThreadPool.
class ThreadPoolLayout : public OSD_ThreadPool
{
public:
  static constexpr size_t THE_THREAD_SIZE = sizeof(OSD_ThreadPool::EnumeratedThread);
};

} // namespace

// Binary compatibility guard for the out-of-memory flag of OSD_ThreadPool::EnumeratedThread.
// The flag is a bool placed after three other bool fields, in the tail padding of the class,
// so the size of EnumeratedThread, and of Launcher that holds one by value, did not change.
// The sizes are those of the MSVC x64 release build of upstream V8_0_1.
TEST(OSD_ParallelTest, ThreadPoolLayoutIsUnchanged)
{
  std::cout << "sizeof(OSD_ThreadPool::EnumeratedThread) = " << ThreadPoolLayout::THE_THREAD_SIZE
            << ", sizeof(OSD_ThreadPool::Launcher) = " << sizeof(OSD_ThreadPool::Launcher)
            << std::endl;
#if defined(_MSC_VER) && defined(_WIN64) && !defined(_DEBUG)
  static_assert(ThreadPoolLayout::THE_THREAD_SIZE == 432, "EnumeratedThread size changed");
  static_assert(sizeof(OSD_ThreadPool::Launcher) == 480, "Launcher size changed");
#endif
}
