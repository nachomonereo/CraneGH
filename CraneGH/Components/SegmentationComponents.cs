using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_SharpFeatureCreases : GH_Component
    {
        public GH_SharpFeatureCreases()
          : base("Sharp Feature Creases", "SharpEdges",
              "Detects strong dihedral feature edges (aristas fuertes) and normal discontinuities on a mesh to serve as segmentation or diffusion boundaries.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.secondary;

        protected override Bitmap Icon => CraneIcons.GetIcon("SharpEdges");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Crease Angle", "A", "Minimum dihedral angle in degrees to classify an edge as a strong feature crease", GH_ParamAccess.item, 28.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Crease Curves", "C", "Joined feature crease polylines where dihedral angle >= A", GH_ParamAccess.list);
            pManager.AddNumberParameter("Vertex Sharpness", "S", "Maximum incident dihedral angle (in radians) per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Feature Heatmap Mesh", "CM", "Mesh colored by dihedral feature strength", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            double angleDeg = 28.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref angleDeg);

            var ddg = new DdgMesh(m);
            double threshRad = Math.Max(0.01, angleDeg * Math.PI / 180.0);

            var sharpSegments = new List<Line>();
            foreach (var e in ddg.Edges)
            {
                if (e.DihedralAngle >= threshRad)
                    sharpSegments.Add(new Line(ddg.Positions[e.V0], ddg.Positions[e.V1]));
            }

            var joinedPolylines = new List<Polyline>();
            if (sharpSegments.Count > 0)
            {
                Curve[] crvs = Curve.JoinCurves(sharpSegments.ConvertAll(s => (Curve)new LineCurve(s)), ddg.MeanEdgeLength * 1e-3);
                if (crvs != null)
                {
                    foreach (var c in crvs)
                        if (c.TryGetPolyline(out Polyline pl))
                            joinedPolylines.Add(pl);
                }
            }

            Mesh colored = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, ddg.VertexSharpness, false);

            DA.SetDataList(0, joinedPolylines);
            DA.SetDataList(1, ddg.VertexSharpness);
            DA.SetData(2, colored);
        }

        public override Guid ComponentGuid => new Guid("A108B208-C308-4D08-9E08-000000000023");
    }

    public class GH_FeatureAwareHeatmap : GH_Component
    {
        public GH_FeatureAwareHeatmap()
          : base("Feature-Aware Heatmap", "FeatHeat",
              "Computes an anisotropic heatmap on a mesh where strong dihedral edges (aristas fuertes) and normal changes act as clear thermal barriers.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("FeatHeat");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Source Points", "S", "Heat source points on the mesh", GH_ParamAccess.list);
            pManager.AddNumberParameter("Barrier Strength", "B", "Strength of normal & sharp-edge barrier (0 = isotropic heat, 5..15 = strong stop at sharp creases)", GH_ParamAccess.item, 6.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Feature Heatmap", "H", "Feature-aware heatmap scalar in [0, 1] per vertex", GH_ParamAccess.list);
            pManager.AddCurveParameter("Region Boundary", "C", "Iso-contour boundary curve at H = 0.5", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the feature-aware heatmap", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var pts = new List<Point3d>();
            double barrier = 6.0;
            if (!DA.GetData(0, ref m) || m == null) return;
            if (!DA.GetDataList(1, pts) || pts.Count == 0) return;
            DA.GetData(2, ref barrier);

            var ddg = new DdgMesh(m);
            var srcVerts = new List<int>(pts.Count);
            foreach (var p in pts)
                srcVerts.Add(ddg.FindClosestVertex(p));

            double[] heat = DdgAlgorithms.ComputeFeatureAwareHeatmap(ddg, srcVerts, barrier, 2.0);
            List<Polyline> boundary = ddg.ExtractIsoContours(heat, new double[] { 0.5 });
            Mesh colored = DdgAlgorithms.ColorMeshByScalar(ddg.RhinoMesh, heat, false);

            DA.SetDataList(0, heat);
            DA.SetDataList(1, boundary);
            DA.SetData(2, colored);
        }

        public override Guid ComponentGuid => new Guid("A108B208-C308-4D08-9E08-000000000024");
    }

    public class GH_MeshHeatmapSegmenter : GH_Component
    {
        public GH_MeshHeatmapSegmenter()
          : base("Mesh Heatmap Segmenter", "MeshSeg",
              "Segments a mesh into K distinct pieces based on vertex Position proximity, Normal similarity, and Sharp Feature Creases (aristas fuertes) as natural boundaries.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override Bitmap Icon => CraneIcons.GetIcon("MeshSeg");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh to segment", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Piece Count", "K", "Number of mesh pieces / segments to generate", GH_ParamAccess.item, 6);
            pManager.AddNumberParameter("Crease & Normal Weight", "B", "Strength of Normal similarity & Sharp Edge barriers vs Position distance (higher = strictly respects sharp creases)", GH_ParamAccess.item, 4.0);
            int idxW = pManager.AddNumberParameter("Vertex Weights", "W", "Optional per-vertex weights (e.g. Gaussian Curvature K folds/pliegues) to attract segmentation boundaries", GH_ParamAccess.list);
            pManager[idxW].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Sub-Meshes", "SM", "Segmented independent mesh pieces", GH_ParamAccess.list);
            pManager.AddCurveParameter("Boundary Cuts", "BC", "Segmentation boundary curves (connect to FlipOut Geodesic Network Shorten to tension the network!)", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Face Labels", "ID", "Segment index (0..K-1) per triangle face", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Combined mesh colored by segment piece", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            int kCount = 6;
            double weight = 4.0;
            var vWeights = new List<double>();
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref kCount);
            DA.GetData(2, ref weight);
            DA.GetDataList(3, vWeights);

            var ddg = new DdgMesh(m);
            List<Mesh> subMeshes = DdgAlgorithms.ComputeMeshHeatmapSegmentation(
                ddg, kCount, null, weight, vWeights.Count > 0 ? vWeights : null,
                out List<Polyline> cuts, out int[] labels, out Mesh colored);

            DA.SetDataList(0, subMeshes);
            DA.SetDataList(1, cuts);
            DA.SetDataList(2, labels);
            DA.SetData(3, colored);
        }

        public override Guid ComponentGuid => new Guid("A108B208-C308-4D08-9E08-000000000025");
    }

    public class GH_VariationalSurfaceCuts : GH_Component
    {
        public GH_VariationalSurfaceCuts()
          : base("Variational Surface Cuts", "VarCuts",
              "Computes optimal seam cuts on a mesh driven by distortion and optional per-vertex weights (such as Gaussian curvature folds/pliegues) to guide where cuts settle (Sharp & Crane, SIGGRAPH 2018).",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("VarCuts");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input 3D mesh", GH_ParamAccess.item);
            int idxW = pManager.AddNumberParameter("Vertex Weights", "W", "Optional per-vertex weights (e.g. Gaussian Curvature K or fold weights) to attract cuts to creases/pliegues", GH_ParamAccess.list);
            pManager[idxW].Optional = true;
            pManager.AddIntegerParameter("Cut Contours", "N", "Number of variational cut contours to extract", GH_ParamAccess.item, 1);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Variational Cuts", "C", "Optimal variational cut curves attracted to folds/high-curvature creases", GH_ParamAccess.list);
            pManager.AddNumberParameter("Distortion Field", "U", "Variational Yamabe distortion potential per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by variational cut potential", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh m = null;
            var weights = new List<double>();
            int nCuts = 1;
            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, weights);
            DA.GetData(2, ref nCuts);

            var ddg = new DdgMesh(m);
            List<Polyline> cuts = DdgAlgorithms.ComputeVariationalSurfaceCuts(
                ddg, weights.Count > 0 ? weights : null, nCuts, out double[] pot, out Mesh colored);

            DA.SetDataList(0, cuts);
            DA.SetDataList(1, pot);
            DA.SetData(2, colored);
        }

        public override Guid ComponentGuid => new Guid("A108B208-C308-4D08-9E08-000000000028");
    }

    public class GH_KMeansNormalVectorSegmenter : GH_Component
    {
        private readonly List<Line> _clusterNormalArrows = new List<Line>();
        private readonly List<Point3d> _clusterCenters = new List<Point3d>();
        private readonly List<Polyline> _seams = new List<Polyline>();

        public GH_KMeansNormalVectorSegmenter()
          : base("K-Means Normal & Vector Segmenter", "KMeansSeg",
              "Segments a mesh into K pieces using Variational K-Means++ clustering combining Face Normals, Tangent Vector Field, Spatial Position, and Sharp Crease/Scalar Features. Outputs optimal Mold Pull / CNC Indexing Normals per cluster.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("KMeansSeg");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh to segment", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Cluster Count", "K", "Number of K-Means clusters / segments", GH_ParamAccess.item, 6);
            int idxVF = pManager.AddVectorParameter("Vector Field", "VF", "Optional tangent vector field (per vertex or face) to include in clustering", GH_ParamAccess.list);
            pManager[idxVF].Optional = true;
            pManager.AddNumberParameter("Normal & Crease Weight", "W", "Weight of Face Normal similarity and Sharp Crease barriers vs Position compactness", GH_ParamAccess.item, 3.5);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Sub-Meshes", "SM", "Segmented mesh pieces", GH_ParamAccess.list);
            pManager.AddVectorParameter("Cluster Pull Normals", "PN", "Representative unit Normal / Pull Direction for each cluster (use for Multi-Piece Molds or 5-Axis CNC indexing!)", GH_ParamAccess.list);
            pManager.AddPointParameter("Cluster Centroids", "CP", "Area-weighted 3D centroid of each cluster", GH_ParamAccess.list);
            pManager.AddCurveParameter("Boundary Seams", "BC", "Segmentation boundary curves between clusters", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Face Labels", "ID", "Cluster index (0..K-1) per triangle face", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Combined mesh colored by K-Means cluster", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _clusterNormalArrows.Clear();
            _clusterCenters.Clear();
            _seams.Clear();

            Mesh m = null;
            int kCount = 6;
            double wN = 3.5;
            var vecField = new List<Vector3d>();

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref kCount);
            DA.GetDataList(2, vecField);
            DA.GetData(3, ref wN);

            var ddg = new DdgMesh(m);
            List<Mesh> subMeshes = DdgAlgorithms.ComputeKMeansNormalVectorSegmentation(
                ddg, kCount, wN, 1.5, 1.5, wN * 0.6,
                vecField.Count > 0 ? vecField : null,
                null,
                out Vector3d[] pullNormals,
                out Point3d[] centroids,
                out List<Polyline> boundaryCurves,
                out int[] faceLabels,
                out Mesh coloredMesh);

            double arrowLen = Math.Max(ddg.MeanEdgeLength * 3.0, m.GetBoundingBox(false).Diagonal.Length * 0.12);
            for (int c = 0; c < pullNormals.Length; c++)
            {
                if (pullNormals[c].Length > 1e-12)
                {
                    _clusterCenters.Add(centroids[c]);
                    _clusterNormalArrows.Add(new Line(centroids[c], centroids[c] + arrowLen * pullNormals[c]));
                }
            }
            _seams.AddRange(boundaryCurves);

            DA.SetDataList(0, subMeshes);
            DA.SetDataList(1, pullNormals);
            DA.SetDataList(2, centroids);
            DA.SetDataList(3, boundaryCurves);
            DA.SetDataList(4, faceLabels);
            DA.SetData(5, coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            foreach (var pl in _seams)
                args.Display.DrawPolyline(pl, Color.FromArgb(30, 30, 35), 2);

            for (int i = 0; i < _clusterNormalArrows.Count; i++)
            {
                args.Display.DrawArrow(_clusterNormalArrows[i], Color.Gold);
                args.Display.DrawDot(_clusterCenters[i], $"M{i}", Color.FromArgb(35, 40, 55), Color.White);
            }
        }

        public override Guid ComponentGuid => new Guid("A108B208-C308-4D08-9E08-000000000037");
    }
}
