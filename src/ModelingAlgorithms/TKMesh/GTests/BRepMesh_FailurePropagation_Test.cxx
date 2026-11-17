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
// and any other Standard_Failure caught during face meshing sets IMeshData_Failure.

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
#include <IMeshTools_ModelAlgo.hxx>
#include <IMeshTools_ModelBuilder.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <Standard_OutOfMemory.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>

#include <gtest/gtest.h>

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

//! Face meshing algorithm that fails inside generateMesh().
class FailingMeshAlgo : public BRepMesh_DelaunayBaseMeshAlgo
{
public:
  explicit FailingMeshAlgo(const bool theIsOutOfMemory)
      : myIsOutOfMemory(theIsOutOfMemory)
  {
  }

protected:
  void generateMesh(const Message_ProgressRange&) override
  {
    throwInjectedFailure(myIsOutOfMemory);
  }

private:
  bool myIsOutOfMemory;
};

//! Factory returning FailingMeshAlgo for every surface type.
class FailingMeshAlgoFactory : public IMeshTools_MeshAlgoFactory
{
public:
  explicit FailingMeshAlgoFactory(const bool theIsOutOfMemory)
      : myIsOutOfMemory(theIsOutOfMemory)
  {
  }

  occ::handle<IMeshTools_MeshAlgo> GetAlgo(const GeomAbs_SurfaceType,
                                           const IMeshTools_Parameters&) const override
  {
    return new FailingMeshAlgo(myIsOutOfMemory);
  }

private:
  bool myIsOutOfMemory;
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
                     const occ::handle<IMeshTools_Context>& theContext)
{
  theMesher.SetShape(theShape);
  theMesher.ChangeParameters() = sequentialParameters();
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
}

TEST(BRepMesh_FailurePropagationTest, EdgeDiscretizationOutOfMemoryReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingEdgeModelBuilder(true));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
}

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
}

TEST(BRepMesh_FailurePropagationTest, ModelBuilderOutOfMemoryReachesCaller)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

  occ::handle<BRepMesh_Context> aContext = new BRepMesh_Context();
  aContext->SetModelBuilder(new FailingModelBuilder(true));

  BRepMesh_IncrementalMesh aMesher;
  EXPECT_THROW(meshWithContext(aMesher, aBox, aContext), Standard_OutOfMemory);
}

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
