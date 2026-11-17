// Created on: 1995-06-20
// Created by: Stagiaire Alain JOURDAIN
// Copyright (c) 1995-1999 Matra Datavision
// Copyright (c) 1999-2014 OPEN CASCADE SAS
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

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_TFace.hxx>
#include <BRepMesh_Context.hxx>
#include <IMeshData_Edge.hxx>
#include <IMeshData_Face.hxx>
#include <IMeshData_Wire.hxx>
#include <IMeshTools_MeshBuilder.hxx>
#include <Standard_OutOfMemory.hxx>
#include <TopExp_Explorer.hxx>

#include <new>
#include <unordered_set>
#include <utility>
#include <vector>

IMPLEMENT_STANDARD_RTTIEXT(BRepMesh_IncrementalMesh, BRepMesh_DiscretRoot)

namespace
{
//! Default flag to control parallelization for BRepMesh_IncrementalMesh
//! tool returned for Mesh Factory
static bool IS_IN_PARALLEL = false;

//! Returns true if the representation holds polygonal (mesh) data of an edge.
bool isPolygon(const occ::handle<BRep_CurveRepresentation>& theCurve)
{
  return theCurve->IsPolygon3D() || theCurve->IsPolygonOnTriangulation()
         || theCurve->IsPolygonOnSurface();
}

//! Keeps the triangulations of the faces and the polygons of the edges of a shape,
//! so that a meshing run that ran out of memory leaves the previous mesh on the shape.
//! Save() allocates; Restore() does not, so that it works in an out-of-memory condition.
class MeshStateKeeper
{
public:
  //! Saves the mesh data of each face and edge of the shape.
  //! Only handles are copied: the triangulations and polygons are shared with the shape.
  void Save(const TopoDS_Shape& theShape)
  {
    std::unordered_set<const TopoDS_TShape*> aVisited;
    for (TopExp_Explorer aFaceIt(theShape, TopAbs_FACE); aFaceIt.More(); aFaceIt.Next())
    {
      const occ::handle<BRep_TFace> aTFace =
        occ::down_cast<BRep_TFace>(aFaceIt.Current().TShape());
      if (!aTFace.IsNull() && aVisited.insert(aTFace.get()).second)
      {
        myFaces.emplace_back();
        FaceState& aState     = myFaces.back();
        aState.TFace          = aTFace;
        aState.Triangulations = aTFace->Triangulations();
        aState.Active         = aTFace->ActiveTriangulation();
      }
    }

    for (TopExp_Explorer anEdgeIt(theShape, TopAbs_EDGE); anEdgeIt.More(); anEdgeIt.Next())
    {
      const occ::handle<BRep_TEdge> aTEdge =
        occ::down_cast<BRep_TEdge>(anEdgeIt.Current().TShape());
      if (aTEdge.IsNull() || !aVisited.insert(aTEdge.get()).second)
      {
        continue;
      }

      myEdges.emplace_back();
      EdgeState& aState = myEdges.back();
      aState.TEdge      = aTEdge;
      for (NCollection_List<occ::handle<BRep_CurveRepresentation>>::Iterator aCurveIt(
             aTEdge->Curves());
           aCurveIt.More();
           aCurveIt.Next())
      {
        if (isPolygon(aCurveIt.Value()))
        {
          aState.HasPolygons = true;
          break;
        }
      }
      if (!aState.HasPolygons)
      {
        // Nothing to keep: restoring removes the polygons added by the run.
        continue;
      }

      // The mesher changes polygonal representations in place, so these are copied;
      // a copy shares the polygons. The other representations are kept as they are.
      for (NCollection_List<occ::handle<BRep_CurveRepresentation>>::Iterator aCurveIt(
             aTEdge->Curves());
           aCurveIt.More();
           aCurveIt.Next())
      {
        const occ::handle<BRep_CurveRepresentation>& aCurve = aCurveIt.Value();
        aState.Curves.Append(isPolygon(aCurve) ? aCurve->Copy() : aCurve);
      }
    }
  }

  //! Puts the saved mesh data back on the faces and edges. Does not allocate memory.
  void Restore() noexcept
  {
    for (FaceState& aState : myFaces)
    {
      aState.TFace->ExchangeTriangulations(aState.Triangulations, aState.Active);

      // Replacing the active triangulation during the run has reset its flag.
      const occ::handle<Poly_Triangulation>& anActive = aState.TFace->ActiveTriangulation();
      if (!anActive.IsNull())
      {
        anActive->SetMeshPurpose(anActive->MeshPurpose() | Poly_MeshPurpose_Active);
      }
    }
    myFaces.clear();

    for (EdgeState& aState : myEdges)
    {
      NCollection_List<occ::handle<BRep_CurveRepresentation>>& aCurves =
        aState.TEdge->ChangeCurves();
      if (aState.HasPolygons)
      {
        aCurves.Exchange(aState.Curves);
        continue;
      }

      NCollection_List<occ::handle<BRep_CurveRepresentation>>::Iterator aCurveIt(aCurves);
      while (aCurveIt.More())
      {
        if (isPolygon(aCurveIt.Value()))
        {
          aCurves.Remove(aCurveIt);
        }
        else
        {
          aCurveIt.Next();
        }
      }
    }
    myEdges.clear();
  }

private:
  struct FaceState
  {
    occ::handle<BRep_TFace>                           TFace;
    NCollection_List<occ::handle<Poly_Triangulation>> Triangulations;
    occ::handle<Poly_Triangulation>                   Active;
  };

  struct EdgeState
  {
    occ::handle<BRep_TEdge>                                 TEdge;
    NCollection_List<occ::handle<BRep_CurveRepresentation>> Curves;
    bool                                                    HasPolygons = false;
  };

  std::vector<FaceState> myFaces;
  std::vector<EdgeState> myEdges;
};
} // namespace

//=================================================================================================

BRepMesh_IncrementalMesh::BRepMesh_IncrementalMesh()
    : myModified(false),
      myStatus(IMeshData_NoError)
{
}

//=================================================================================================

BRepMesh_IncrementalMesh::BRepMesh_IncrementalMesh(const TopoDS_Shape& theShape,
                                                   const double        theLinDeflection,
                                                   const bool          isRelative,
                                                   const double        theAngDeflection,
                                                   const bool          isInParallel)
    : myModified(false),
      myStatus(IMeshData_NoError)
{
  myParameters.Deflection = theLinDeflection;
  myParameters.Angle      = theAngDeflection;
  myParameters.Relative   = isRelative;
  myParameters.InParallel = isInParallel;

  myShape = theShape;
  Perform();
}

//=================================================================================================

BRepMesh_IncrementalMesh::BRepMesh_IncrementalMesh(const TopoDS_Shape&          theShape,
                                                   const IMeshTools_Parameters& theParameters,
                                                   const Message_ProgressRange& theRange)
    : myParameters(theParameters)
{
  myShape = theShape;
  Perform(theRange);
}

//=================================================================================================

BRepMesh_IncrementalMesh::~BRepMesh_IncrementalMesh() = default;

//=================================================================================================

void BRepMesh_IncrementalMesh::Perform(const Message_ProgressRange& theRange)
{
  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context(myParameters.MeshAlgo);
  Perform(aContext, theRange);
}

//=================================================================================================

void BRepMesh_IncrementalMesh::Perform(const occ::handle<IMeshTools_Context>& theContext,
                                       const Message_ProgressRange&           theRange)
{
  // Do not report the result of a previous run.
  myStatus = IMeshData_NoError;
  setNotDone();
  initParameters();

  theContext->SetShape(Shape());
  theContext->ChangeParameters()            = myParameters;
  theContext->ChangeParameters().CleanModel = false;

  Message_ProgressScope  aPS(theRange, "Perform incmesh", 10);
  IMeshTools_MeshBuilder aIncMesh(theContext);

  // A run that runs out of memory leaves the mesh that the shape had before it:
  // the stages remove outdated triangulations and polygons before the new ones are complete.
  MeshStateKeeper aInitialMesh;
  try
  {
    aInitialMesh.Save(Shape());
    aIncMesh.Perform(aPS.Next(9));
  }
  catch (Standard_OutOfMemory const&)
  {
    aInitialMesh.Restore();
    myStatus = IMeshData_Failure;
    throw;
  }
  catch (std::bad_alloc const&)
  {
    aInitialMesh.Restore();
    myStatus = IMeshData_Failure;
    throw Standard_OutOfMemory("BRepMesh_IncrementalMesh::Perform: std::bad_alloc");
  }
  if (!aPS.More())
  {
    myStatus = IMeshData_UserBreak;
    return;
  }

  // A failed stage (model building, edge discretization, healing, pre- or
  // post-processing, face meshing) is a failure of the whole run.
  if (aIncMesh.GetStatus().IsFail())
  {
    myStatus |= IMeshData_Failure;
  }

  const occ::handle<IMeshData_Model>& aModel = theContext->GetModel();
  if (!aModel.IsNull())
  {
    for (int aEdgeIt = 0; aEdgeIt < aModel->EdgesNb(); ++aEdgeIt)
    {
      if (aModel->GetEdge(aEdgeIt)->IsSet(IMeshData_Failure))
      {
        myStatus |= IMeshData_Failure;
      }
    }

    for (int aFaceIt = 0; aFaceIt < aModel->FacesNb(); ++aFaceIt)
    {
      const IMeshData::IFaceHandle& aDFace = aModel->GetFace(aFaceIt);
      myStatus |= aDFace->GetStatusMask();

      for (int aWireIt = 0; aWireIt < aDFace->WiresNb(); ++aWireIt)
      {
        const IMeshData::IWireHandle& aDWire = aDFace->GetWire(aWireIt);
        myStatus |= aDWire->GetStatusMask();
      }
    }
  }
  aPS.Next(1);
  setDone();
}

//=================================================================================================

int BRepMesh_IncrementalMesh::Discret(const TopoDS_Shape&    theShape,
                                      const double           theDeflection,
                                      const double           theAngle,
                                      BRepMesh_DiscretRoot*& theAlgo)
{
  BRepMesh_IncrementalMesh* anAlgo      = new BRepMesh_IncrementalMesh();
  anAlgo->ChangeParameters().Deflection = theDeflection;
  anAlgo->ChangeParameters().Angle      = theAngle;
  anAlgo->ChangeParameters().InParallel = IS_IN_PARALLEL;
  anAlgo->SetShape(theShape);
  theAlgo = anAlgo;
  return 0; // no error
}

//=================================================================================================

bool BRepMesh_IncrementalMesh::IsParallelDefault()
{
  return IS_IN_PARALLEL;
}

//=================================================================================================

void BRepMesh_IncrementalMesh::SetParallelDefault(const bool theInParallel)
{
  IS_IN_PARALLEL = theInParallel;
}
