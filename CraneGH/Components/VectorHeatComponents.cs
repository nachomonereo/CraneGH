using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_VectorParallelTransport : GH_Component
    {
        public GH_VectorParallelTransport()
          : base("Vector Parallel Transport", "VecHeat",
              "Parallel-transports tangent vectors across a curved mesh using Keenan Crane's Vector Heat Method (ACM TOG 2019).",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("VecHeat");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Source Points", "P", "Points on the mesh where tangent vectors are prescribed", GH_ParamAccess.list);
            pManager.AddVectorParameter("Source Vectors", "V", "Tangent vectors at the source points", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddVectorParameter("Vector Field", "VF", "Parallel-transported tangent vector per mesh vertex", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var pts = new List<Point3d>();
            var vecs = new List<Vector3d>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, pts) || pts.Count == 0) return;
            if (!DA.GetDataList(2, vecs) || vecs.Count == 0) return;

            var ddg = new DdgMesh(m);
            var srcVerts = new List<int>(pts.Count);
            foreach (var p in pts)
                srcVerts.Add(ddg.FindClosestVertex(p));

            Vector3d[] transported = DdgAlgorithms.ComputeVectorHeatTransport(ddg, srcVerts, vecs, 1.0);
            DA.SetDataList(0, transported);
        }

        public override Guid ComponentGuid => new Guid("A103B203-C303-4D03-9E03-000000000008");
    }

    public class GH_LogarithmicMap : GH_Component
    {
        public GH_LogarithmicMap()
          : base("Logarithmic Map UV", "LogMap",
              "Computes Geodesic Polar Coordinates (Logarithmic Map) around an origin point on the mesh via the Vector Heat Method.",
              "CraneGH", "03. Vector & Direction Fields")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override Bitmap Icon => CraneIcons.GetIcon("LogMap");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Origin", "O", "Center point of the Logarithmic Map on the mesh", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "D", "Reference tangent direction for angle theta = 0", GH_ParamAccess.item, Vector3d.XAxis);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("UV Coordinates", "UV", "Local geodesic Cartesian coordinates (r*cos(theta), r*sin(theta), 0) per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Radius", "R", "Geodesic distance r from Origin per vertex", GH_ParamAccess.list);
            pManager.AddNumberParameter("Angle", "A", "Geodesic polar angle theta [-pi, pi] per vertex", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            Point3d origin = Point3d.Origin;
            Vector3d dir = Vector3d.XAxis;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetData(1, ref origin)) return;
            DA.GetData(2, ref dir);

            var ddg = new DdgMesh(m);
            int vOrig = ddg.FindClosestVertex(origin);
            Point3d[] uv = DdgAlgorithms.ComputeLogarithmicMap(ddg, vOrig, dir, out double[] radii, out double[] angles);

            DA.SetDataList(0, uv);
            DA.SetDataList(1, radii);
            DA.SetDataList(2, angles);
        }

        public override Guid ComponentGuid => new Guid("A103B203-C303-4D03-9E03-000000000009");
    }

    public class GH_SurfaceDecalMapper : GH_Component
    {
        public GH_SurfaceDecalMapper()
          : base("Surface Decal Mapper", "DecalMap",
              "Maps 2D curves from the XY plane onto a 3D mesh using vertex UV coordinates (from Logarithmic Map or Conformal Flatten BFF).",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override Bitmap Icon => CraneIcons.GetIcon("DecalMap");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Target 3D surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Vertex UV", "UV", "2D parameter coordinates per mesh vertex (from LogMap or BFF)", GH_ParamAccess.list);
            pManager.AddCurveParameter("2D Curves", "C", "Planar curves in XY space to wrap onto the 3D mesh", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("3D Curves", "C3D", "Curves mapped onto the 3D surface mesh", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uvPts = new List<Point3d>();
            var crvs2D = new List<Curve>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uvPts) || uvPts.Count == 0) return;
            if (!DA.GetDataList(2, crvs2D) || crvs2D.Count == 0) return;

            var ddg = new DdgMesh(m);
            Mesh flatParamMesh = ddg.RhinoMesh.DuplicateMesh();
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                Point3d p = uvPts[Math.Min(i, uvPts.Count - 1)];
                flatParamMesh.Vertices.SetVertex(i, p.X, p.Y, 0.0);
            }

            var outCrvs = new List<Polyline>();
            foreach (Curve c in crvs2D)
            {
                if (c == null) continue;
                Point3d[] divPts;
                c.DivideByCount(64, true, out divPts);
                if (divPts == null || divPts.Length < 2) continue;

                var mappedPts = new List<Point3d>(divPts.Length);
                foreach (Point3d p2 in divPts)
                {
                    MeshPoint mp = flatParamMesh.ClosestMeshPoint(new Point3d(p2.X, p2.Y, 0.0), 0.0);
                    if (mp != null)
                    {
                        MeshFace f = ddg.RhinoMesh.Faces[mp.FaceIndex];
                        Point3d p3 = mp.T[0] * ddg.Positions[f.A] +
                                     mp.T[1] * ddg.Positions[f.B] +
                                     mp.T[2] * ddg.Positions[f.C];
                        mappedPts.Add(p3);
                    }
                }
                if (mappedPts.Count >= 2)
                    outCrvs.Add(new Polyline(mappedPts));
            }

            DA.SetDataList(0, outCrvs);
        }

        public override Guid ComponentGuid => new Guid("A103B203-C303-4D03-9E03-000000000010");
    }
}
