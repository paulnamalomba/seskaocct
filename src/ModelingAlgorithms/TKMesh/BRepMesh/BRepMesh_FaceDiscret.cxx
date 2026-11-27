// Created on: 2016-04-19
// Copyright (c) 2016 OPEN CASCADE SAS
// Created by: Oleg AGASHIN
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

#include <BRepMesh_FaceDiscret.hxx>
#include <NCollection_LinearVector.hxx>
#include <IMeshData_Model.hxx>
#include <IMeshData_Wire.hxx>
#include <IMeshData_Edge.hxx>
#include <IMeshTools_MeshAlgo.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Message.hxx>
#include <OSD_Parallel.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_OutOfMemory.hxx>

#include <atomic>
#include <exception>
#include <new>
#include <vector>

IMPLEMENT_STANDARD_RTTIEXT(BRepMesh_FaceDiscret, IMeshTools_ModelAlgo)

//=================================================================================================

BRepMesh_FaceDiscret::BRepMesh_FaceDiscret(
  const occ::handle<IMeshTools_MeshAlgoFactory>& theAlgoFactory)
    : myAlgoFactory(theAlgoFactory)
{
}

//=================================================================================================

BRepMesh_FaceDiscret::~BRepMesh_FaceDiscret() = default;

//! Auxiliary functor for parallel processing of Faces.
class BRepMesh_FaceDiscret::FaceListFunctor
{
public:
  FaceListFunctor(BRepMesh_FaceDiscret* theAlgo, const Message_ProgressRange& theRange)
      : myAlgo(theAlgo),
        myScope(theRange, "Face Discret", theAlgo->myModel->FacesNb())
  {
    myRanges.Reserve(theAlgo->myModel->FacesNb());
    for (int aFaceIter = 0; aFaceIter < theAlgo->myModel->FacesNb(); ++aFaceIter)
    {
      myRanges.Append(myScope.Next());
    }
  }

  void operator()(const int theFaceIndex) const
  {
    if (!myScope.More() || myIsOutOfMemory.load())
    {
      return;
    }
    Message_ProgressScope aFaceScope(myRanges[theFaceIndex], nullptr, 1);
    try
    {
      myAlgo->process(theFaceIndex, aFaceScope.Next());
    }
    catch (Standard_OutOfMemory const&)
    {
      // Keep the first one, skip the remaining faces and let the caller
      // rethrow it after the loop, when no thread works on the model.
      keepOutOfMemory(std::current_exception());
    }
    catch (std::bad_alloc const&)
    {
      keepOutOfMemory(std::make_exception_ptr(
        Standard_OutOfMemory("BRepMesh_FaceDiscret: std::bad_alloc")));
    }
  }

  //! Rethrows Standard_OutOfMemory raised while processing a face, if any.
  void RethrowOutOfMemory() const
  {
    if (myHasOutOfMemory.load())
    {
      std::rethrow_exception(myOutOfMemory);
    }
  }

  //! Returns true if processing of a face ran out of memory.
  bool IsOutOfMemory() const { return myHasOutOfMemory.load(); }

private:
  //! Keeps the first out-of-memory exception and makes the remaining faces be skipped.
  void keepOutOfMemory(const std::exception_ptr& theException) const
  {
    bool isFirst = false;
    if (myIsOutOfMemory.compare_exchange_strong(isFirst, true))
    {
      myOutOfMemory = theException;
      myHasOutOfMemory.store(true);
    }
  }

private:
  mutable BRepMesh_FaceDiscret*                   myAlgo;
  Message_ProgressScope                           myScope;
  NCollection_LinearVector<Message_ProgressRange> myRanges;
  mutable std::atomic<bool>                       myIsOutOfMemory{false};  //!< claimed by a thread
  mutable std::atomic<bool>                       myHasOutOfMemory{false}; //!< exception is stored
  mutable std::exception_ptr                      myOutOfMemory;
};

//=================================================================================================

bool BRepMesh_FaceDiscret::performInternal(const occ::handle<IMeshData_Model>& theModel,
                                           const IMeshTools_Parameters&        theParameters,
                                           const Message_ProgressRange&        theRange)
{
  //! Releases the link to the model on every exit path.
  class ModelReleaser
  {
  public:
    ModelReleaser(occ::handle<IMeshData_Model>& theModel)
        : myModel(theModel)
    {
    }

    ~ModelReleaser() { myModel.Nullify(); }

  private:
    occ::handle<IMeshData_Model>& myModel;
  };

  const ModelReleaser aReleaser(myModel);
  myModel      = theModel;
  myParameters = theParameters;
  if (myModel.IsNull())
  {
    return false;
  }

  // Triangulations of the faces before meshing, restored if meshing runs out of memory.
  const int                                    aFacesNb = myModel->FacesNb();
  std::vector<occ::handle<Poly_Triangulation>> aInitialTriangulations(aFacesNb);
  for (int aFaceIt = 0; aFaceIt < aFacesNb; ++aFaceIt)
  {
    const TopoDS_Face& aFace = myModel->GetFace(aFaceIt)->GetFace();
    TopLoc_Location    aLoc;
    if (!aFace.IsNull())
    {
      aInitialTriangulations[aFaceIt] = BRep_Tool::Triangulation(aFace, aLoc);
    }
  }

  FaceListFunctor aFunctor(this, theRange);
  OSD_Parallel::For(0, aFacesNb, aFunctor, !myParameters.InParallel || aFacesNb <= 1);
  if (aFunctor.IsOutOfMemory())
  {
    // Do not leave the triangulations of the faces meshed before the failure on the shape.
    // This is a best effort for direct users of BRepMesh_FaceDiscret: BRep_Builder::UpdateFace
    // can allocate, and the polygons of the edges are not handled here. The defined state after
    // out of memory is the one restored by BRepMesh_IncrementalMesh::Perform.
    BRep_Builder aBuilder;
    for (int aFaceIt = 0; aFaceIt < aFacesNb; ++aFaceIt)
    {
      const TopoDS_Face& aFace = myModel->GetFace(aFaceIt)->GetFace();
      TopLoc_Location    aLoc;
      if (!aFace.IsNull()
          && BRep_Tool::Triangulation(aFace, aLoc) != aInitialTriangulations[aFaceIt])
      {
        aBuilder.UpdateFace(aFace, aInitialTriangulations[aFaceIt]);
      }
    }
    aFunctor.RethrowOutOfMemory();
  }

  return theRange.More();
}

//=================================================================================================

void BRepMesh_FaceDiscret::process(const int                    theFaceIndex,
                                   const Message_ProgressRange& theRange) const
{
  const IMeshData::IFaceHandle& aDFace = myModel->GetFace(theFaceIndex);
  if (aDFace->IsSet(IMeshData_Failure) || aDFace->IsSet(IMeshData_Reused))
  {
    return;
  }

  try
  {
    OCC_CATCH_SIGNALS

    occ::handle<IMeshTools_MeshAlgo> aMeshingAlgo =
      myAlgoFactory->GetAlgo(aDFace->GetSurface()->GetType(), myParameters);

    if (aMeshingAlgo.IsNull())
    {
      aDFace->SetStatus(IMeshData_Failure);
      return;
    }

    if (!theRange.More())
    {
      aDFace->SetStatus(IMeshData_UserBreak);
      return;
    }
    aMeshingAlgo->Perform(aDFace, myParameters, theRange);
  }
  catch (Standard_OutOfMemory const&)
  {
    aDFace->SetStatus(IMeshData_Failure);
    throw;
  }
  catch (std::bad_alloc const&)
  {
    aDFace->SetStatus(IMeshData_Failure);
    throw Standard_OutOfMemory("BRepMesh_FaceDiscret: std::bad_alloc");
  }
  catch (Standard_Failure const& theException)
  {
    aDFace->SetStatus(IMeshData_Failure);
    Message::SendFail() << "BRepMesh_FaceDiscret: " << theException.ExceptionType() << ": "
                        << theException.what();
  }
}
