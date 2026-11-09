using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_HeatDistance : GH_Component
    {
        private readonly List<Point3d> _sourcePts = new List<Point3d>();

        public GH_HeatDistance()
          : base("Heat Geodesic Distance", "HeatDist",
              "Computes geodesic distance and smoothed gradient on a mesh from ANY external geometry (Points, Curves, Meshes, or Breps) using Keenan Crane's Heat Method (ACM TOG 2013).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("HeatDist");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            int idxG = pManager.AddGeometryParameter("Source Geometry", "G", "External heat sources: Points, Curves, Meshes, or Breps (leave empty to radiate from mesh boundary)", GH_ParamAccess.list);
            pManager[idxG].Optional = true;
            pManager.AddNumberParameter("Smooth Time", "t", "Heat diffusion & gradient smoothing multiplier (default = 1.0)", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Distance", "d", "Geodesic distance per vertex in real Rhino units", GH_ParamAccess.list);
            pManager.AddVectorParameter("Smooth Gradient", "G", "Redistributed smooth unit geodesic gradient vector per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by Keenan Crane's Heat Method colormap", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _sourcePts.Clear();
            Mesh m = null;
            var geoms = new List<GeometryBase>();
            double tFactor = 1.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, geoms);
            DA.GetData(2, ref tFactor);

            var ddg = new DdgMesh(m);
            double[] dist = DdgAlgorithms.ComputeExternalGeometryHeatDistance(
                ddg, geoms, tFactor,
                out Vector3d[] vertGrad,
                out _,
                out List<Point3d> srcPts,
                out Mesh colored);

            _sourcePts.AddRange(srcPts);

            DA.SetDataList(0, dist);
            DA.SetDataList(1, vertGrad);
            DA.SetData(2, colored);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _sourcePts.Count == 0) return;
            args.Display.DrawPoints(_sourcePts, Rhino.Display.PointStyle.RoundSimple, 4, Color.Gold);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000005");
    }

    public class GH_MeshIsoContours : GH_Component
    {
        public GH_MeshIsoContours()
          : base("Mesh Iso-Contours", "IsoCrvs",
              "Extracts smooth iso-contour polylines across a mesh from any vertex scalar field (e.g., Heat Distance, Feature Heatmap, Harmonic Field, Stripe Wave, Curvature).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("IsoCrvs");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Scalar Field", "u", "Scalar value per mesh vertex (can be real distances, coordinates, curvature, or any custom numbers)", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Count", "N", "Number of uniformly spaced iso-contour levels (ignored if Exact Iso-Values V are provided)", GH_ParamAccess.item, 20);
            int idxV = pManager.AddNumberParameter("Exact Iso-Values", "V", "Optional exact scalar value(s) to extract (e.g. 4.55 in real distance units, or 0.5 if scalar is 0..1)", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Iso-Curves", "C", "Extracted iso-contour polylines on the mesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("Iso-Levels", "L", "Scalar iso-values at which contours were extracted", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var uList = new List<double>();
            int count = 20;
            var exactVals = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, uList) || uList.Count == 0) return;
            DA.GetData(2, ref count);
            DA.GetDataList(3, exactVals);

            var ddg = new DdgMesh(m);
            double[] u = ddg.MapScalarInputToVertices(uList);
            int n = ddg.VertexCount;

            double minU = double.MaxValue, maxU = double.MinValue;
            for (int i = 0; i < n; i++)
            {
                if (u[i] < minU) minU = u[i];
                if (u[i] > maxU) maxU = u[i];
            }

            var levels = new List<double>();
            if (exactVals.Count > 0)
            {
                levels.AddRange(exactVals);
            }
            else if (count <= 1)
            {
                levels.Add(minU < 0.0 && maxU > 0.0 ? 0.0 : 0.5 * (minU + maxU));
            }
            else
            {
                for (int k = 1; k <= count; k++)
                {
                    double frac = (double)k / (count + 1);
                    levels.Add(minU + frac * (maxU - minU));
                }
            }

            List<Polyline> contours = ddg.ExtractIsoContours(u, levels);
            DA.SetDataList(0, contours);
            DA.SetDataList(1, levels);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000006");
    }

    public class GH_GeodesicShortestPath : GH_Component
    {
        public GH_GeodesicShortestPath()
          : base("Geodesic Shortest Path", "GeoPath",
              "Traces exact shortest geodesic paths on a mesh from target points down the negative gradient of a Heat Distance field.",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GeoPath");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Distance Field", "d", "Geodesic distance per vertex (from Heat Geodesic Distance)", GH_ParamAccess.list);
            pManager.AddPointParameter("Targets", "T", "Target points from which to trace geodesic paths back to sources", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic Paths", "P", "Traced geodesic shortest path polylines on the mesh", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var dList = new List<double>();
            var targets = new List<Point3d>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, dList) || dList.Count == 0) return;
            if (!DA.GetDataList(2, targets) || targets.Count == 0) return;

            var ddg = new DdgMesh(m);
            double[] d = ddg.MapScalarInputToVertices(dList);
            int n = ddg.VertexCount;

            ddg.ComputeGradient(d, out _, out Vector3d[] gradV);
            double stepSize = 0.45 * ddg.MeanEdgeLength;
            int maxSteps = Math.Max(200, n * 2);

            var paths = new List<Polyline>();
            foreach (Point3d tPt in targets)
            {
                var pts = new List<Point3d>();
                MeshPoint mp = ddg.RhinoMesh.ClosestMeshPoint(tPt, 0.0);
                if (mp == null) continue;

                Point3d curr = mp.Point;
                pts.Add(curr);

                for (int step = 0; step < maxSteps; step++)
                {
                    int vNear = ddg.FindClosestVertex(curr);
                    if (d[vNear] < stepSize * 0.8)
                    {
                        pts.Add(ddg.Positions[vNear]);
                        break;
                    }

                    MeshFace f = ddg.RhinoMesh.Faces[mp.FaceIndex];
                    Vector3d g = mp.T[0] * gradV[f.A] + mp.T[1] * gradV[f.B] + mp.T[2] * gradV[f.C];
                    if (g.Length < 1e-12)
                    {
                        int bestNb = vNear;
                        foreach (int heIdx in ddg.VertexOutgoingHalfEdges[vNear])
                        {
                            int nb = ddg.HalfEdges[heIdx].TargetVertex;
                            if (d[nb] < d[bestNb]) bestNb = nb;
                        }
                        if (bestNb == vNear) break;
                        curr = ddg.Positions[bestNb];
                        pts.Add(curr);
                        mp = ddg.RhinoMesh.ClosestMeshPoint(curr, 0.0);
                        continue;
                    }

                    g.Unitize();
                    Point3d nextCandidate = curr - stepSize * g;
                    mp = ddg.RhinoMesh.ClosestMeshPoint(nextCandidate, 0.0);
                    if (mp == null || mp.Point.DistanceToSquared(curr) < 1e-14)
                        break;

                    curr = mp.Point;
                    pts.Add(curr);
                }

                if (pts.Count >= 2)
                    paths.Add(new Polyline(pts));
            }

            DA.SetDataList(0, paths);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000007");
    }

    public class GH_ExactIsoCut : GH_Component
    {
        private readonly List<Polyline> _cutCurves = new List<Polyline>();

        public GH_ExactIsoCut()
          : base("Iso-Splitting Curves", "IsoCut",
              "Extracts fast Iso-Splitting Curves on a mesh from any Scalar Field, Smoothed Scalar/Mesh, or Scalar Gradient (without slicing the mesh).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("IsoCut");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh (or Colored Mesh CM from SmoothScalar)", GH_ParamAccess.item);
            int idxU = pManager.AddGenericParameter("Scalar or Gradient", "u", "Smoothed Scalar values (double) or Scalar Gradient vectors (Vector3d) (if empty, reads from Colored Mesh M or boundary heat distance)", GH_ParamAccess.list);
            pManager[idxU].Optional = true;
            pManager.AddNumberParameter("Iso-Values or Count", "V", "If a single integer >= 2 (e.g. 12), extracts that many evenly-spaced iso-curves; otherwise extracts at exact value(s) V", GH_ParamAccess.list, 12.0);
            pManager.AddBooleanParameter("Normalized [0..1]", "N", "True = V is evaluated as normalized fraction(s) in [0..1]; False = V is in real scalar units (or line count if single integer >= 2)", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Iso-Splitting Curves", "C", "Extracted iso-contour curves on the mesh (no mesh cutting overhead)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Resolved Scalar", "S", "Active scalar values per vertex used for the iso-lines", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the active scalar field", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _cutCurves.Clear();

            Mesh m = null;
            var rawU = new List<object>();
            var isoVals = new List<double>();
            bool normalized = false;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, rawU);
            if (!DA.GetDataList(2, isoVals) || isoVals.Count == 0) isoVals.Add(12.0);
            DA.GetData(3, ref normalized);

            var ddg = new DdgMesh(m);
            var inScalars = new List<double>();
            var inVecs = new List<Vector3d>();

            foreach (object obj in rawU)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;
                if (val is double d) inScalars.Add(d);
                else if (val is int iv) inScalars.Add(iv);
                else if (val is Vector3d vec) inVecs.Add(vec);
            }

            double[] u;
            if (inScalars.Count > 0)
            {
                u = ddg.MapScalarInputToVertices(inScalars);
            }
            else if (inVecs.Count > 0)
            {
                DdgAlgorithms.SmoothVectorAndGradientMultiMode(
                    ddg, inVecs, null, 2.0, 1, out _, out _, out u, out _);
            }
            else if (m.VertexColors.Count == m.Vertices.Count)
            {
                // Decode smoothed scalar from Colored Mesh CM (e.g. from SmoothScalar) and smooth slightly
                double[] rawCol = ddg.MapScalarInputToVertices(null);
                u = DdgAlgorithms.SmoothScalarField(ddg, rawCol, 1.5, 0.0, out _, out _);
            }
            else
            {
                var seeds = new List<int>();
                for (int i = 0; i < ddg.VertexCount; i++)
                    if (ddg.VertexOnBoundary[i]) seeds.Add(i);
                if (seeds.Count == 0) seeds.Add(0);
                u = DdgAlgorithms.ComputeHeatDistance(ddg, seeds, 1.0, out _, out _);
            }

            double minU = double.MaxValue, maxU = double.MinValue;
            for (int i = 0; i < ddg.VertexCount; i++)
            {
                if (u[i] < minU) minU = u[i];
                if (u[i] > maxU) maxU = u[i];
            }
            double spanU = Math.Max(1e-12, maxU - minU);

            var levels = new List<double>();
            if (!normalized && isoVals.Count == 1 && isoVals[0] >= 2.0 && Math.Abs(isoVals[0] - Math.Round(isoVals[0])) < 1e-5 && (isoVals[0] > maxU || minU >= 0.0))
            {
                // Treat single integer >= 2 (e.g. 12) as number of evenly-spaced iso-splitting lines!
                int count = Math.Min(250, (int)Math.Round(isoVals[0]));
                for (int k = 1; k <= count; k++)
                {
                    double frac = k / (double)(count + 1);
                    levels.Add(minU + frac * spanU);
                }
            }
            else
            {
                foreach (double val in isoVals)
                {
                    levels.Add(normalized ? (minU + val * spanU) : val);
                }
            }
            levels.Sort();

            List<Polyline> crvs = ddg.ExtractIsoContours(u, levels);
            _cutCurves.AddRange(crvs);
            Mesh coloredM = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, u, false);

            DA.SetDataList(0, crvs);
            DA.SetDataList(1, u);
            DA.SetData(2, coloredM);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _cutCurves.Count == 0) return;

            Color cCurve = Attributes.Selected ? Color.FromArgb(255, 255, 80) : Color.FromArgb(255, 210, 40);
            foreach (var pl in _cutCurves)
                args.Display.DrawPolyline(pl, cCurve, 2);
        }

        public override Guid ComponentGuid => new Guid("A102B202-C302-4D02-9E02-000000000034");
    }
}
