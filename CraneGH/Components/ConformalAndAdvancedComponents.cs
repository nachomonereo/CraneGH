using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_SpinConformalDeform : GH_Component
    {
        public GH_SpinConformalDeform()
          : base("Spin Conformal Deform", "SpinXForm",
              "Deforms a 3D mesh by prescribing Mean Curvature changes while preserving local angles via Quaternion Dirac Spin Transformations (Crane, Pinkall, Schröder 2011).",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("SpinXForm");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Curvature Change", "Rho", "Prescribed Mean Curvature change scalar (per face, per vertex, or single multiplier)", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Deformed Mesh", "M", "Conformally deformed 3D mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Spinor Norms", "Q", "Local similarity transformation quaternion magnitude per vertex", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var rhoList = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, rhoList) || rhoList.Count == 0) return;

            var ddg = new DdgMesh(m);
            Mesh deformed = DdgAlgorithms.ComputeSpinTransformation(ddg, rhoList, out double[] norms);

            DA.SetData(0, deformed);
            DA.SetDataList(1, norms);
        }

        public override Guid ComponentGuid => new Guid("A106B206-C306-4D06-9E06-000000000015");
    }

    public class GH_ConformalFlatten : GH_Component
    {
        public GH_ConformalFlatten()
          : base("Conformal Flatten (BFF)", "BFF",
              "Flattens a 3D mesh to the XY plane with minimal angle/area distortion using Boundary First Flattening (Sawhney & Crane 2017).",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("BFF");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "3D surface mesh to flatten", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Boundary Mode", "B", "0 = Natural Free Boundary (minimal area distortion), 1 = Circular Disk Boundary", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Flattened Mesh", "M2D", "2D flattened mesh in the XY plane", GH_ParamAccess.item);
            pManager.AddPointParameter("UV Points", "UV", "2D vertex coordinates (u, v, 0) for Decal / Pattern mapping", GH_ParamAccess.list);
            pManager.AddNumberParameter("Scale Factor", "u", "Conformal logarithmic scale factor per vertex", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            int mode = 0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref mode);

            var ddg = new DdgMesh(m);
            Mesh flatMesh = DdgAlgorithms.ComputeConformalFlattening(ddg, mode, out double[] scaleFactors);

            var uvPts = new List<Point3d>(flatMesh.Vertices.Count);
            for (int i = 0; i < flatMesh.Vertices.Count; i++)
                uvPts.Add(flatMesh.Vertices.Point3dAt(i));

            DA.SetData(0, flatMesh);
            DA.SetDataList(1, uvPts);
            DA.SetDataList(2, scaleFactors);
        }

        public override Guid ComponentGuid => new Guid("A106B206-C306-4D06-9E06-000000000016");
    }

    public class GH_DeployableAuxetics : GH_Component
    {
        public GH_DeployableAuxetics()
          : base("Deployable Auxetic Grid", "Auxetics",
              "Generates a flat 2D rotating-square kirigami cutting pattern and its deployed 3D auxetic linkage on a conformally flattened surface (Konaković, Crane et al. 2016).",
              "CraneGH", "04. Stripes & Conformal")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("Auxetics");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("3D Mesh", "M", "Target 3D doubly-curved mesh", GH_ParamAccess.item);
            pManager.AddMeshParameter("2D Flat Mesh", "M2D", "Flattened 2D mesh from Conformal Flatten (BFF)", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Divisions", "D", "Grid divisions across the 2D bounding domain", GH_ParamAccess.item, 12);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("2D Auxetic Cells", "C2D", "Flat 2D auxetic tiles in the XY plane", GH_ParamAccess.list);
            pManager.AddCurveParameter("3D Deployed Cells", "C3D", "Deployed 3D auxetic rotating-square tiles on the target surface", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m3dRaw = null;
            Mesh m2d = null;
            int div = 12;
            if (!DA.GetData(0, ref m3dRaw) || m3dRaw == null) return;
            if (!DA.GetData(1, ref m2d) || m2d == null) return;
            DA.GetData(2, ref div);

            // Ensure m3d has matching welded topology with m2d
            Mesh m3d = (m3dRaw.Vertices.Count == m2d.Vertices.Count && m3dRaw.Faces.Count == m2d.Faces.Count)
                ? m3dRaw
                : new DdgMesh(m3dRaw).RhinoMesh;

            div = Math.Max(2, Math.Min(100, div));
            BoundingBox bbox = m2d.GetBoundingBox(true);
            double dx = (bbox.Max.X - bbox.Min.X) / div;
            double dy = (bbox.Max.Y - bbox.Min.Y) / div;
            double step = Math.Max(dx, dy);
            if (step < 1e-8) return;

            var cells2D = new List<Polyline>();
            var cells3D = new List<Polyline>();

            for (int ix = 0; ix < div; ix++)
            {
                for (int iy = 0; iy < div; iy++)
                {
                    double cx = bbox.Min.X + (ix + 0.5) * step;
                    double cy = bbox.Min.Y + (iy + 0.5) * step;
                    Point3d center2D = new Point3d(cx, cy, 0.0);

                    MeshPoint mpCenter = m2d.ClosestMeshPoint(center2D, step * 0.35);
                    if (mpCenter == null) continue;

                    MeshFace f = m2d.Faces[mpCenter.FaceIndex];
                    double area2D = TriangleArea(m2d.Vertices.Point3dAt(f.A), m2d.Vertices.Point3dAt(f.B), m2d.Vertices.Point3dAt(f.C));
                    double area3D = TriangleArea(m3d.Vertices.Point3dAt(f.A), m3d.Vertices.Point3dAt(f.B), m3d.Vertices.Point3dAt(f.C));
                    double ratio = Math.Sqrt(Math.Max(0.25, Math.Min(2.0, area3D / Math.Max(1e-12, area2D))));

                    double sign = ((ix + iy) % 2 == 0) ? 1.0 : -1.0;
                    double theta = sign * 0.35 * Math.Min(1.0, Math.Max(0.0, ratio - 0.5));
                    double half = 0.42 * step;

                    Point3d[] corners2D = new Point3d[5];
                    Point3d[] corners3D = new Point3d[5];
                    bool allValid = true;

                    double[,] offsets = new double[,] { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
                    for (int k = 0; k < 4; k++)
                    {
                        double ox = offsets[k, 0] * half;
                        double oy = offsets[k, 1] * half;
                        double rx = ox * Math.Cos(theta) - oy * Math.Sin(theta);
                        double ry = ox * Math.Sin(theta) + oy * Math.Cos(theta);
                        Point3d pt2 = new Point3d(cx + rx, cy + ry, 0.0);
                        corners2D[k] = pt2;

                        MeshPoint mp = m2d.ClosestMeshPoint(pt2, step);
                        if (mp == null) { allValid = false; break; }
                        MeshFace mf = m3d.Faces[mp.FaceIndex];
                        corners3D[k] = mp.T[0] * m3d.Vertices.Point3dAt(mf.A) +
                                       mp.T[1] * m3d.Vertices.Point3dAt(mf.B) +
                                       mp.T[2] * m3d.Vertices.Point3dAt(mf.C);
                    }

                    if (allValid)
                    {
                        corners2D[4] = corners2D[0];
                        corners3D[4] = corners3D[0];
                        cells2D.Add(new Polyline(corners2D));
                        cells3D.Add(new Polyline(corners3D));
                    }
                }
            }

            DA.SetDataList(0, cells2D);
            DA.SetDataList(1, cells3D);
        }

        private static double TriangleArea(Point3d a, Point3d b, Point3d c)
        {
            return 0.5 * Vector3d.CrossProduct(b - a, c - a).Length;
        }

        public override Guid ComponentGuid => new Guid("A106B206-C306-4D06-9E06-000000000017");
    }

    public class GH_GeodesicLoopShorten : GH_Component
    {
        public GH_GeodesicLoopShorten()
          : base("Geodesic Network Shorten", "FlipOut",
              "Shortens curves, loops, or multi-curve networks on a mesh into taut geodesics; shared junction nodes and endpoints lying on other curves slide and tension automatically (Sharp & Crane 2020).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("FlipOut");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Surface mesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Curves", "C", "Curve, closed loop, or network of curves on/near the mesh (junctions and T-endpoints slide on connected curves)", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Iterations", "I", "Geodesic tensioning iterations", GH_ParamAccess.item, 35);
            int idxW = pManager.AddNumberParameter("Vertex Weights", "W", "Optional per-vertex weights (e.g. Gaussian Curvature K folds/pliegues) to attract curves toward folds", GH_ParamAccess.list);
            pManager[idxW].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Geodesic Network", "G", "Shortened geodesic polylines on the mesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("Total Length", "L", "Total geodesic network length", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var crvs = new List<Curve>();
            int iter = 35;
            var weights = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, crvs) || crvs.Count == 0) return;
            DA.GetData(2, ref iter);
            DA.GetDataList(3, weights);

            List<Polyline> geoNet = DdgAlgorithms.ComputeGeodesicNetworkShorten(
                m, crvs, iter, true, out double totalLen, weights.Count > 0 ? weights : null);

            DA.SetDataList(0, geoNet);
            DA.SetData(1, totalLen);
        }

        public override Guid ComponentGuid => new Guid("A107B207-C307-4D07-9E07-000000000018");
    }

    public class GH_RepulsiveCurveFlow : GH_Component
    {
        public GH_RepulsiveCurveFlow()
          : base("Repulsive Curve Flow", "RepCurve",
              "Untangles and separates 3D curves/knots using Tangent-Point Repulsive Energy (Yu, Schumacher, Crane, ACM TOG 2021).",
              "CraneGH", "02. Geodesics & Iso-Cut")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override Bitmap Icon => CraneIcons.GetIcon("RepCurve");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Curve", "C", "Input 3D curve or knot", GH_ParamAccess.item);
            pManager.AddNumberParameter("Step Size", "S", "Repulsive flow step size", GH_ParamAccess.item, 0.15);
            pManager.AddIntegerParameter("Iterations", "I", "Number of repulsive flow iterations", GH_ParamAccess.item, 25);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Repulsive Curve", "C", "Untangled / self-avoiding polyline curve", GH_ParamAccess.item);
            pManager.AddNumberParameter("Tangent-Point Energy", "E", "Final Tangent-Point repulsive energy", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Curve crv = null;
            double step = 0.15;
            int iter = 25;
            if (!DA.GetData(0, ref crv) || crv == null) return;
            DA.GetData(1, ref step);
            DA.GetData(2, ref iter);

            if (!crv.TryGetPolyline(out Polyline pl))
            {
                Point3d[] pts;
                crv.DivideByCount(64, true, out pts);
                pl = new Polyline(pts);
                if (crv.IsClosed && pts.Length > 0)
                    pl.Add(pts[0]);
            }

            Polyline outPl = DdgAlgorithms.ComputeRepulsiveCurveFlow(pl, step, iter, true, out double energy);
            DA.SetData(0, outPl);
            DA.SetData(1, energy);
        }

        public override Guid ComponentGuid => new Guid("A107B207-C307-4D07-9E07-000000000019");
    }

    public class GH_WalkOnSpheres : GH_Component
    {
        public GH_WalkOnSpheres()
          : base("Walk on Spheres PDE", "WoS",
              "Mesh-free Monte Carlo Laplace PDE solver in 3D space (Sawhney & Crane, SIGGRAPH 2020).",
              "CraneGH", "01. Operators & Display")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("WoS");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Boundary Mesh", "M", "Boundary mesh (can have holes or self-intersections)", GH_ParamAccess.item);
            pManager.AddNumberParameter("Boundary Values", "BV", "Dirichlet scalar values at boundary mesh vertices", GH_ParamAccess.list);
            pManager.AddPointParameter("Query Points", "P", "Points in 3D space where the Laplace solution is evaluated", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Solved Values", "U", "Monte Carlo Laplace solution values at Query Points", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var bVals = new List<double>();
            var qPts = new List<Point3d>();
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, bVals) || bVals.Count == 0) return;
            if (!DA.GetDataList(2, qPts) || qPts.Count == 0) return;

            double[] res = DdgAlgorithms.ComputeWalkOnSpheres3D(m, bVals, qPts, 64, 16, 1e-2);
            DA.SetDataList(0, res);
        }

        public override Guid ComponentGuid => new Guid("A107B207-C307-4D07-9E07-000000000020");
    }

    public class GH_SubgridMarchingTet : GH_Component
    {
        public GH_SubgridMarchingTet()
          : base("Subgrid Marching Tet", "SubgridTet",
              "Reconstructs an intersection-free multi-crossing surface triangulation inside a tetrahedron from its 6 edge crossing counts (Baktash, Gillespie, Crane, ACM TOG 2026).",
              "CraneGH", "01. Operators & Display")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("SubgridTet");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Tet Vertices", "V", "4 corner points of the tetrahedron (v0, v1, v2, v3)", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Edge Crossings", "E", "6 non-negative crossing counts in canonical order (e01, e02, e03, e23, e13, e12)", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Reconstructed Mesh", "M", "Intersection-free triangulated surface inside the tetrahedron", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var verts = new List<Point3d>();
            var crossings = new List<int>();
            if (!DA.GetDataList(0, verts) || verts.Count < 4) return;
            if (!DA.GetDataList(1, crossings) || crossings.Count < 6) return;

            Point3d[] tetPts = new Point3d[] { verts[0], verts[1], verts[2], verts[3] };
            int[] edgeCounts = new int[] { crossings[0], crossings[1], crossings[2], crossings[3], crossings[4], crossings[5] };

            Mesh mesh = DdgAlgorithms.ReconstructSubgridTetrahedron(tetPts, edgeCounts);
            DA.SetData(0, mesh);
        }

        public override Guid ComponentGuid => new Guid("A107B207-C307-4D07-9E07-000000000021");
    }
}
