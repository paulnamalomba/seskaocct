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

// Failures raised inside meshing algorithms must reach the caller:
// Standard_OutOfMemory propagates out of BRepMesh_IncrementalMesh::Perform(),
// and any other Standard_Failure caught during meshing sets IMeshData_Failure.

#include <BRep_Tool.hxx>
#include <BRepMesh_Context.hxx>
#include <BRepMesh_DelaunayBaseMeshAlgo.hxx>
#include <BRepMesh_FaceDiscret.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepMesh_ModelBuilder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <IMeshData_Curve.hxx>
#include <IMeshData_Edge.hxx>
#include <IMeshData_Model.hxx>
#include <IMeshData_Status.hxx>
#include <IMeshTools_MeshAlgoFactory.hxx>
#include <IMeshTools_MeshBuilder.hxx>
#include <IMeshTools_ModelAlgo.hxx>
#include <IMeshTools_ModelBuilder.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_Printer.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <Standard_NullObject.hxx>
#include <Standard_OutOfMemory.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>

#include <gtest/gtest.h>

#include <atomic>
#include <new>

namespace
{

//! Throws Standard_OutOfMemory or a plain Standard_Failure, as selected.
[[noreturn]] void throwInjectedFailure(const bool theIsOutOfMemory)
{
  if (theIsOutOfMemory)
  {
    throw Standard_OutOfMemory("injected out-of-memory condition");
  }
  throw Standard_Failure("injected failure");
}

//! Delaunay face meshing algorithm that fails inside generateMesh() when asked to.
class FailingMeshAlgo : public BRepMesh_DelaunayBaseMeshAlgo
{
public:
  FailingMeshAlgo(const bool theIsFailing, const bool theIsOutOfMemory)
      : myIsFailing(theIsFailing),
        myIsOutOfMemory(theIsOutOfMemory)
  {
  }

protected:
  void generateMesh(const Message_ProgressRange& theRange) override
  {
    if (myIsFailing)
    {
      throwInjectedFailure(myIsOutOfMemory);
    }
    BRepMesh_DelaunayBaseMeshAlgo::generateMesh(theRange);
  }

private:
  bool myIsFailing;
  bool myIsOutOfMemory;
};

//! Factory returning FailingMeshAlgo for every surface type.
//! The algorithm fails for every face, or only for the face meshed in the given order.
class FailingMeshAlgoFactory : public IMeshTools_MeshAlgoFactory
{
public:
  explicit FailingMeshAlgoFactory(const bool theIsOutOfMemory, const int theFailingAlgo = -1)
      : myIsOutOfMemory(theIsOutOfMemory),
        myFailingAlgo(theFailingAlgo)
  {
  }

  occ::handle<IMeshTools_MeshAlgo> GetAlgo(const GeomAbs_SurfaceType,
                                           const IMeshTools_Parameters&) const override
  {
    const int  anAlgoIndex = myAlgosNb++;
    const bool isFailing   = myFailingAlgo < 0 || anAlgoIndex == myFailingAlgo;
    return new FailingMeshAlgo(isFailing, myIsOutOfMemory);
  }

private:
  bool                     myIsOutOfMemory;
  int                      myFailingAlgo;
  mutable std::atomic<int> myAlgosNb{0};
};

//! Discrete 3d curve whose every access fails.
class FailingCurve : public IMeshData_Curve
{
public:
  explicit FailingCurve(const bool theIsOutOfMemory)
      : myIsOutOfMemory(theIsOutOfMemory)
  {
  }

  void InsertPoint(const int, const gp_Pnt&, const double) override
  {
    throwInjectedFailure(myIsOutOfMemory);
  }

  void AddPoint(const gp_Pnt&, const double) override { throwInjectedFailure(myIsOutOfMemory); }

  gp_Pnt& GetPoint(const int) override { throwInjectedFailure(myIsOutOfMemory); }

  void RemovePoint(const int) override { throwInjectedFailure(myIsOutOfMemory); }

  double& GetParameter(const int) override { throwInjectedFailure(myIsOutOfMemory); }

  int ParametersNb() const override { throwInjectedFailure(myIsOutOfMemory); }

  void Clear(const bool) override { throwInjectedFailure(myIsOutOfMemory); }

protected:
  void removeParameter(const int) override { throwInjectedFailure(myIsOutOfMemory); }

private:
  bool myIsOutOfMemory;
};

//! Builds the default discrete model, then replaces the 3d curve of the first edge
//! by FailingCurve so that edge discretization fails.
class FailingEdgeModelBuilder : public IMeshTools_ModelBuilder
{
public:
  explicit FailingEdgeModelBuilder(const bool theIsOutOfMemory)
      : myIsOutOfMemory(theIsOutOfMemory)
  {
  }

protected:
  occ::handle<IMeshData_Model> performInternal(const TopoDS_Shape&          theShape,
                                               const IMeshTools_Parameters& theParameters) override
  {
    occ::handle<IMeshTools_ModelBuilder> aBuilder = new BRepMesh_ModelBuilder();
    occ::handle<IMeshData_Model>         aModel   = aBuilder->Perform(theShape, theParameters);
    if (!aModel.IsNull() && aModel->EdgesNb() > 0)
    {
      aModel->GetEdge(0)->SetCurve(new FailingCurve(myIsOutOfMemory));
    }
    return aModel;
  }

private:
  bool myIsOutOfMemory;
};

//! Model builder that fails before creating a model.
class FailingModelBuilder : public IMeshTools_ModelBuilder
{
public:
  explicit FailingModelBuilder(const bool theIsOutOfMemory)
      : myIsOutOfMemory(theIsOutOfMemory)
  {
  }

protected:
  occ::handle<IMeshData_Model> performInternal(const TopoDS_Shape&,
                                               const IMeshTools_Parameters&) override
  {
    throwInjectedFailure(myIsOutOfMemory);
  }

private:
  bool myIsOutOfMemory;
};

//! Model algorithm that fails when performed.
class FailingModelAlgo : public IMeshTools_ModelAlgo
{
public:
  explicit FailingModelAlgo(const bool theIsOutOfMemory)
      : myIsOutOfMemory(theIsOutOfMemory)
  {
  }

protected:
  bool performInternal(const occ::handle<IMeshData_Model>&,
                       const IMeshTools_Parameters&,
                       const Message_ProgressRange&) override
  {
    throwInjectedFailure(myIsOutOfMemory);
  }

private:
  bool myIsOutOfMemory;
};

//! Delaunay face meshing algorithm that throws std::bad_alloc inside generateMesh().
class BadAllocMeshAlgo : public BRepMesh_DelaunayBaseMeshAlgo
{
protected:
  void generateMesh(const Message_ProgressRange&) override { throw std::bad_alloc(); }
};

//! Factory returning BadAllocMeshAlgo for every surface type.
class BadAllocMeshAlgoFactory : public IMeshTools_MeshAlgoFactory
{
public:
  occ::handle<IMeshTools_MeshAlgo> GetAlgo(const GeomAbs_SurfaceType,
                                           const IMeshTools_Parameters&) const override
  {
    return new BadAllocMeshAlgo();
  }
};

//! Model builder that throws std::bad_alloc before creating a model.
class BadAllocModelBuilder : public IMeshTools_ModelBuilder
{
protected:
  occ::handle<IMeshData_Model> performInternal(const TopoDS_Shape&,
                                               const IMeshTools_Parameters&) override
  {
    throw std::bad_alloc();
  }
};

//! Model algorithm that throws std::bad_alloc when performed.
class BadAllocModelAlgo : public IMeshTools_ModelAlgo
{
protected:
  bool performInternal(const occ::handle<IMeshData_Model>&,
                       const IMeshTools_Parameters&,
                       const Message_ProgressRange&) override
  {
    throw std::bad_alloc();
  }
};

//! Collects the texts of the failure messages sent to the default messenger.
class FailMessageCollector : public Message_Printer
{
public:
  const TCollection_AsciiString& Text() const { return myText; }

protected:
  void send(const TCollection_AsciiString& theString,
            const Message_Gravity          theGravity) const override
  {
    if (theGravity == Message_Fail)
    {
      myText += theString;
      myText += "\n";
    }
  }

private:
  mutable TCollection_AsciiString myText;
};

//! Attaches a FailMessageCollector to the default messenger for its lifetime.
class FailMessageScope
{
public:
  FailMessageScope()
      : myCollector(new FailMessageCollector())
  {
    Message::DefaultMessenger()->AddPrinter(myCollector);
  }

  ~FailMessageScope() { Message::DefaultMessenger()->RemovePrinter(myCollector); }

  const TCollection_AsciiString& Text() const { return myCollector->Text(); }

private:
  occ::handle<FailMessageCollector> myCollector;
};

//! Returns meshing parameters for a sequential run.
IMeshTools_Parameters sequentialParameters()
{
  IMeshTools_Parameters aParams;
  aParams.Deflection = 0.1;
  aParams.Angle      = 0.5;
  aParams.InParallel = false;
  return aParams;
}

//! Meshes the shape with the given context.
void meshWithContext(BRepMesh_IncrementalMesh&              theMesher,
                     const TopoDS_Shape&                    theShape,
                     const occ::handle<IMeshTools_Context>& theContext,
                     const bool                             theInParallel = false)
{
  theMesher.SetShape(theShape);
  theMesher.ChangeParameters()            = sequentialParameters();
  theMesher.ChangeParameters().InParallel = theInParallel;
  theMesher.Perform(theContext);
}

//! Returns the number of faces of the shape that have a triangulation.
int triangulatedFacesNb(const TopoDS_Shape& theShape)
{
  int aNb = 0;
  for (TopExp_Explorer anExp(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
  {
    TopLoc_Location aLoc;
    if (!BRep_Tool::Triangulation(TopoDS::Face(anExp.Current()), aLoc).IsNull())
    {
      ++aNb;
    }
  }
  return aNb;
}

} // namespace

TEST(BRepMesh_FailurePropagationTest, FaceMeshingOutOfMemoryReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new FailingMeshAlgoFactory(true)));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
}

TEST(BRepMesh_FailurePropagationTest, FaceMeshingFailureSetsFailureStatus)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new FailingMeshAlgoFactory(false)));

  FailMessageScope         aMessages;
  BRepMesh_IncrementalMesh aMesher;
  ASSERT_NO_THROW(meshWithContext(aMesher, aBox, aContext));

  EXPECT_EQ(0, triangulatedFacesNb(aBox));
  EXPECT_NE(0, aMesher.GetStatusFlags());
  EXPECT_NE(0, aMesher.GetStatusFlags() & IMeshData_Failure)
    << "Status flags: " << aMesher.GetStatusFlags();

  const occ::handle<IMeshData_Model>& aModel = aContext->GetModel();
  ASSERT_FALSE(aModel.IsNull());
  ASSERT_EQ(6, aModel->FacesNb());
  for (int aFaceIt = 0; aFaceIt < aModel->FacesNb(); ++aFaceIt)
  {
    EXPECT_TRUE(aModel->GetFace(aFaceIt)->IsSet(IMeshData_Failure)) << "Face " << aFaceIt;
  }

  // The caught exception is reported with its type and text.
  EXPECT_GT(
    aMessages.Text().Search("BRepMesh_BaseMeshAlgo::Perform: Standard_Failure: injected failure"),
    0)
    << aMessages.Text();
}

TEST(BRepMesh_FailurePropagationTest, FaceMeshingOutOfMemoryInParallelReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new FailingMeshAlgoFactory(true, 3)));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext, true), Standard_OutOfMemory);
  EXPECT_EQ(0, triangulatedFacesNb(aBox));
  EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
}

TEST(BRepMesh_FailurePropagationTest, StateAfterFaceMeshingOutOfMemory)
{
  BRepMesh_IncrementalMesh aMesher;

  // A successful run first, so that a stale result would be visible.
  const TopoDS_Shape aFirstBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();
  meshWithContext(aMesher, aFirstBox, new BRepMesh_Context());
  ASSERT_TRUE(aMesher.IsDone());
  ASSERT_EQ(0, aMesher.GetStatusFlags());

  // The fourth face runs out of memory after three faces have been meshed.
  const TopoDS_Shape            aBox     = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();
  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new FailingMeshAlgoFactory(true, 3)));
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);

  // No partial triangulation is left on the shape.
  EXPECT_EQ(0, triangulatedFacesNb(aBox));

  // BRepMesh_FaceDiscret does not keep a link to the model; only the context holds it.
  ASSERT_FALSE(aContext->GetModel().IsNull());
  EXPECT_EQ(1, aContext->GetModel()->GetRefCount());

  // The result of the first run is not reported.
  EXPECT_FALSE(aMesher.IsDone());
  EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
}

TEST(BRepMesh_FailurePropagationTest, MeshBuilderCleansContextOnOutOfMemory)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetShape(aBox);
  aContext->ChangeParameters()            = sequentialParameters();
  aContext->ChangeParameters().CleanModel = true;
  // Values that BRepMesh_IncrementalMesh would derive from Deflection and Angle.
  aContext->ChangeParameters().DeflectionInterior = 0.1;
  aContext->ChangeParameters().AngleInterior      = 1.0;
  aContext->ChangeParameters().MinSize            = 1.0e-4;
  aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new FailingMeshAlgoFactory(true)));

  IMeshTools_MeshBuilder aBuilder(aContext);
  EXPECT_THROW(aBuilder.Perform(Message_ProgressRange()), Standard_OutOfMemory);
  EXPECT_TRUE(aContext->GetModel().IsNull());
}

TEST(BRepMesh_FailurePropagationTest, EdgeDiscretizationOutOfMemoryReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingEdgeModelBuilder(true));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);

  // BRepMesh_EdgeDiscret does not keep a link to the model after the exception.
  ASSERT_FALSE(aContext->GetModel().IsNull());
  EXPECT_EQ(1, aContext->GetModel()->GetRefCount());
  EXPECT_FALSE(aMesher.IsDone());
  EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
}

// OSD_ThreadPool keeps the exception type when the failing edge is processed in the pool.
TEST(BRepMesh_FailurePropagationTest, EdgeDiscretizationOutOfMemoryInParallelReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingEdgeModelBuilder(true));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext, true), Standard_OutOfMemory);
  EXPECT_FALSE(aMesher.IsDone());
  EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
}

// Regression guard: an edge failure was already flagged, and reached the mesher status
// through the faces bounded by the edge.
TEST(BRepMesh_FailurePropagationTest, EdgeDiscretizationFailureSetsFailureStatus)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingEdgeModelBuilder(false));

  BRepMesh_IncrementalMesh aMesher;
  ASSERT_NO_THROW(meshWithContext(aMesher, aBox, aContext));

  const occ::handle<IMeshData_Model>& aModel = aContext->GetModel();
  ASSERT_FALSE(aModel.IsNull());
  ASSERT_GT(aModel->EdgesNb(), 0);
  EXPECT_TRUE(aModel->GetEdge(0)->IsSet(IMeshData_Failure));
  EXPECT_NE(0, aMesher.GetStatusFlags() & IMeshData_Failure)
    << "Status flags: " << aMesher.GetStatusFlags();
}

TEST(BRepMesh_FailurePropagationTest, ModelBuilderOutOfMemoryReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingModelBuilder(true));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
}

TEST(BRepMesh_FailurePropagationTest, ModelBuilderFailureSetsMesherFailureStatus)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingModelBuilder(false));

  BRepMesh_IncrementalMesh aMesher;
  ASSERT_NO_THROW(meshWithContext(aMesher, aBox, aContext));
  EXPECT_NE(0, aMesher.GetStatusFlags() & IMeshData_Failure)
    << "Status flags: " << aMesher.GetStatusFlags();
}

// Regression guard: IMeshTools_ModelBuilder already reported a plain failure as Message_Fail2.
TEST(BRepMesh_FailurePropagationTest, ModelBuilderFailureSetsFailStatus)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<IMeshTools_ModelBuilder> aBuilder = new FailingModelBuilder(false);
  IMeshTools_Parameters                aParams  = sequentialParameters();

  occ::handle<IMeshData_Model> aModel;
  ASSERT_NO_THROW(aModel = aBuilder->Perform(aBox, aParams));
  EXPECT_TRUE(aModel.IsNull());
  EXPECT_TRUE(aBuilder->GetStatus().IsSet(Message_Fail2));
}

TEST(BRepMesh_FailurePropagationTest, ModelAlgoOutOfMemoryReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelHealer(new FailingModelAlgo(true));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
}

TEST(BRepMesh_FailurePropagationTest, ModelHealerFailureSetsMesherFailureStatus)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelHealer(new FailingModelAlgo(false));

  BRepMesh_IncrementalMesh aMesher;
  ASSERT_NO_THROW(meshWithContext(aMesher, aBox, aContext));
  EXPECT_NE(0, aMesher.GetStatusFlags() & IMeshData_Failure)
    << "Status flags: " << aMesher.GetStatusFlags();
}

TEST(BRepMesh_FailurePropagationTest, PreProcessorFailureSetsMesherFailureStatus)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetPreProcessor(new FailingModelAlgo(false));

  BRepMesh_IncrementalMesh aMesher;
  ASSERT_NO_THROW(meshWithContext(aMesher, aBox, aContext));
  EXPECT_NE(0, aMesher.GetStatusFlags() & IMeshData_Failure)
    << "Status flags: " << aMesher.GetStatusFlags();
}

// Regression guard: IMeshTools_ModelAlgo already returned false for a plain failure.
TEST(BRepMesh_FailurePropagationTest, ModelAlgoFailureReturnsFalse)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<IMeshTools_ModelBuilder> aBuilder = new BRepMesh_ModelBuilder();
  const IMeshTools_Parameters          aParams  = sequentialParameters();
  const occ::handle<IMeshData_Model>   aModel   = aBuilder->Perform(aBox, aParams);
  ASSERT_FALSE(aModel.IsNull());

  occ::handle<IMeshTools_ModelAlgo> anAlgo = new FailingModelAlgo(false);
  bool                              isDone = true;
  ASSERT_NO_THROW(isDone = anAlgo->Perform(aModel, aParams, Message_ProgressRange()));
  EXPECT_FALSE(isDone);
}

TEST(BRepMesh_FailurePropagationTest, NullFaceIsRejected)
{
  occ::handle<IMeshTools_MeshAlgo> anAlgo = new BRepMesh_DelaunayBaseMeshAlgo();
  EXPECT_THROW(
    anAlgo->Perform(IMeshData::IFaceHandle(), sequentialParameters(), Message_ProgressRange()),
    Standard_NullObject);
}

// std::bad_alloc is reported like Standard_OutOfMemory at every stage.
TEST(BRepMesh_FailurePropagationTest, FaceMeshingBadAllocReachesCallerAsOutOfMemory)
{
  for (const bool isInParallel : {false, true})
  {
    const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

    occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
    aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new BadAllocMeshAlgoFactory()));

    BRepMesh_IncrementalMesh aMesher;
    EXPECT_THROW(meshWithContext(aMesher, aBox, aContext, isInParallel), Standard_OutOfMemory)
      << "In parallel: " << isInParallel;
    EXPECT_EQ(0, triangulatedFacesNb(aBox));
    EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
  }
}

TEST(BRepMesh_FailurePropagationTest, ModelBuilderBadAllocReachesCallerAsOutOfMemory)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new BadAllocModelBuilder());

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
  EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
}

TEST(BRepMesh_FailurePropagationTest, ModelAlgoBadAllocReachesCallerAsOutOfMemory)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelHealer(new BadAllocModelAlgo());

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
  EXPECT_EQ(int(IMeshData_Failure), aMesher.GetStatusFlags());
}
