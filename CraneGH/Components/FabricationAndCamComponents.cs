using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using CraneGH.Core;

namespace CraneGH.Components
{
    public class GH_RollingSpheresOffset : GH_Component
    {
        private readonly List<Sphere> _spheres = new List<Sphere>();
        private readonly List<Point3d> _gougePts = new List<Point3d>();

        public GH_RollingSpheresOffset()
          : base("Rolling Spheres & Collision Offset", "RollSphere",
              "Computes Tangent-Point Rolling Spheres (Keenan Crane's Repulsive Surfaces 2021) for CNC Ball-Mill gouge detection (+R exterior), Medial Wall Thickness (-R interior), and collision-free mesh offsetting.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("RollSphere");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input surface mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Signed Radius", "R", "Target Tool / Offset Radius (+R = Exterior CNC Accessibility & Offset; -R = Interior Medial Wall Thickness)", GH_ParamAccess.item, 2.0);
            pManager.AddIntegerParameter("Display Spheres", "N", "Number of tangent rolling spheres to display", GH_ParamAccess.item, 35);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Collision-Free Offset", "OM", "Offset mesh clamped by local rolling-sphere radius so it never self-intersects", GH_ParamAccess.item);
            pManager.AddBrepParameter("Rolling Spheres", "S", "Tangent rolling spheres touching the surface without colliding", GH_ParamAccess.list);
            pManager.AddNumberParameter("Max Rolling Radius", "RMax", "Maximum collision-free rolling sphere radius per vertex", GH_ParamAccess.list);
            pManager.AddPointParameter("Collision / Gouge Points", "G", "Vertices where Max Rolling Radius < |R| (CNC gouging or thin-wall warning)", GH_ParamAccess.list);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by maximum collision-free rolling sphere radius", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _spheres.Clear();
            _gougePts.Clear();

            Mesh m = null;
            double signedRadius = 2.0;
            int nSpheres = 35;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref signedRadius);
            DA.GetData(2, ref nSpheres);

            bool exterior = signedRadius >= 0.0;
            double radius = Math.Abs(signedRadius);

            var ddg = new DdgMesh(m);
            double[] rMax = DdgAlgorithms.ComputeRollingSpheresAndOffset(
                ddg, radius, exterior, nSpheres,
                out List<Sphere> spheres,
                out Mesh offsetMesh,
                out List<Point3d> gougePts,
                out Mesh coloredMesh);

            _spheres.AddRange(spheres);
            _gougePts.AddRange(gougePts);

            DA.SetData(0, offsetMesh);
            DA.SetDataList(1, spheres);
            DA.SetDataList(2, rMax);
            DA.SetDataList(3, gougePts);
            DA.SetData(4, coloredMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            Color sphColor = Color.FromArgb(140, 0, 215, 255);
            foreach (var s in _spheres)
            {
                if (s.IsValid && s.Radius > 1e-6)
                    args.Display.DrawSphere(s, sphColor);
            }

            if (_gougePts.Count > 0)
                args.Display.DrawPoints(_gougePts, Rhino.Display.PointStyle.RoundControlPoint, 4, Color.Crimson);
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000036");
    }

    public class GH_MoldPartingAnalyzer : GH_Component
    {
        private readonly List<Polyline> _partingLines = new List<Polyline>();
        private readonly List<Line> _pullArrows = new List<Line>();
        private readonly List<Point3d> _pieceCenters = new List<Point3d>();
        private readonly List<string> _pieceTags = new List<string>();

        public GH_MoldPartingAnalyzer()
          : base("Multi-Piece Mold & Draft Cut", "MultiMold",
              "Partitions a 3D mesh into K extractable Mold Pieces using Multiple Pull Vectors (or automatic Curvature-driven pull discovery) combined with Heat Geodesic Distance, Draft Angle analysis, and Curvature Parting Line snapping.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("MoldParting");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input 3D mesh to partition into extractable mold pieces", GH_ParamAccess.item);
            int idxD = pManager.AddVectorParameter("Pull Vectors", "D", "Optional mold pull direction vectors (leave empty to automatically discover K pull directions from Mesh Curvature & Geodesic Distance!)", GH_ParamAccess.list);
            pManager[idxD].Optional = true;
            pManager.AddIntegerParameter("Mold Pieces", "K", "Target number of mold pieces (2 = Core/Cavity, 4..12 = Multi-Piece Mold)", GH_ParamAccess.item, 4);
            pManager.AddNumberParameter("Min Draft Angle", "A", "Minimum required draft angle in degrees (regions with draft < A are highlighted as undercuts)", GH_ParamAccess.item, 3.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mold Pieces", "MP", "Extracted contiguous sub-meshes for each of the K mold pieces", GH_ParamAccess.list);
            pManager.AddVectorParameter("Pull Vectors", "PV", "Optimal unit extraction / pull direction vector for each mold piece", GH_ParamAccess.list);
            pManager.AddCurveParameter("Parting Lines", "PL", "Smooth mold parting line curves snapped to curvature ridges", GH_ParamAccess.list);
            pManager.AddNumberParameter("Draft Angles (Deg)", "Deg", "Effective draft angle in degrees [-90..+90] per vertex relative to its mold piece pull vector", GH_ParamAccess.list);
            pManager.AddMeshParameter("Undercut Mesh", "UC", "Sub-mesh of any remaining undercut or low-draft faces (where draft angle < A)", GH_ParamAccess.item);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Combined mesh colored by Mold Piece with Gold/Crimson highlights on low-draft/undercut zones", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _partingLines.Clear();
            _pullArrows.Clear();
            _pieceCenters.Clear();
            _pieceTags.Clear();

            Mesh m = null;
            var pullDirs = new List<Vector3d>();
            int kPieces = 4;
            double draftDeg = 3.0;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, pullDirs);
            DA.GetData(2, ref kPieces);
            DA.GetData(3, ref draftDeg);

            var ddg = new DdgMesh(m);
            List<Mesh> pieces = DdgAlgorithms.ComputeMultiPieceMoldAndDraftCut(
                ddg,
                pullDirs.Count > 0 ? pullDirs : null,
                kPieces,
                draftDeg,
                0.35,
                2.0,
                1.2,
                out Vector3d[] resolvedPulls,
                out Point3d[] centroids,
                out List<Polyline> parting,
                out double[] draftAngles,
                out Mesh undercutMesh,
                out _,
                out Mesh coloredM);

            _partingLines.AddRange(parting);

            BoundingBox bbox = m.GetBoundingBox(false);
            double arrowLen = Math.Max(ddg.MeanEdgeLength * 3.5, bbox.Diagonal.Length * 0.16);
            for (int k = 0; k < resolvedPulls.Length; k++)
            {
                Point3d c = centroids[k];
                Vector3d d = resolvedPulls[k];
                _pieceCenters.Add(c);
                _pullArrows.Add(new Line(c, c + arrowLen * d));
                _pieceTags.Add($"M{k}");
            }

            DA.SetDataList(0, pieces);
            DA.SetDataList(1, resolvedPulls);
            DA.SetDataList(2, parting);
            DA.SetDataList(3, draftAngles);
            DA.SetData(4, undercutMesh);
            DA.SetData(5, coloredM);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            foreach (var pl in _partingLines)
                args.Display.DrawPolyline(pl, Color.Gold, 4);

            for (int i = 0; i < _pullArrows.Count; i++)
            {
                args.Display.DrawArrow(_pullArrows[i], Color.FromArgb(60, 235, 165));
                args.Display.DrawDot(_pieceCenters[i], _pieceTags[i], Color.FromArgb(30, 38, 52), Color.White);
            }
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000038");
    }

    public class GH_ConcentricMillingToolpaths : GH_Component
    {
        private readonly List<Polyline> _toolpaths = new List<Polyline>();
        private readonly List<Plane> _frames = new List<Plane>();
        private double _axisScale = 1.0;

        public GH_ConcentricMillingToolpaths()
          : base("Milling & 3D Print Iso-Paths", "FabPaths",
              "Generates concentric CNC Milling & Curved-Layer 3D Printing paths at exact Stepover/Pitch D from any external source geometry, with Ball-Mill offset and Collision-Smoothed 5/6-Axis Planes.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("MillingPaths");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Surface mesh for CNC Milling or Curved 3D Printing", GH_ParamAccess.item);
            pManager.AddNumberParameter("Stepover / Pitch", "D", "Exact physical stepover or layer distance in Rhino units (e.g. 2.5 or 4.55; 0 = automatic)", GH_ParamAccess.item, 0.0);
            pManager.AddNumberParameter("Tool / Nozzle Offset", "R", "Ball-end mill radius or nozzle standoff offset (computed via Tangent-Point Rolling Spheres!)", GH_ParamAccess.item, 0.0);
            int idxS = pManager.AddGenericParameter("Scalar / Gradient / Source", "S", "Smoothed Scalar Field (double), Scalar Gradient / Vector Field (Vector3d), or External Source Geometry (Curve, Point, Mesh)", GH_ParamAccess.list);
            pManager[idxS].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Surface Iso-Paths", "CP", "Concentric iso-paths on the mesh separated by exact Distance D", GH_ParamAccess.list);
            pManager.AddCurveParameter("Offset Toolpaths", "TP", "3D offset toolpaths computed on the Rolling-Sphere Offset Geometry", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Smooth CNC / Print Planes", "F", "Collision-tilted, smoothed 5/6-Axis Planes without drastic turns", GH_ParamAccess.list);
            pManager.AddMeshParameter("Rolling-Sphere OffsetGeo", "OffsetGeo", "Collision-free Rolling-Sphere Offset Mesh used for the milling toolpath offset", GH_ParamAccess.item);
            pManager.AddMeshParameter("Colored Mesh", "CM", "Mesh colored by the driving scalar or gradient field", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _toolpaths.Clear();
            _frames.Clear();

            Mesh m = null;
            double stepover = 0.0;
            double toolR = 0.0;
            var rawSource = new List<object>();

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref stepover);
            DA.GetData(2, ref toolR);
            DA.GetDataList(3, rawSource);

            var ddg = new DdgMesh(m);
            _axisScale = ddg.MeanEdgeLength * 0.9;

            var inScalars = new List<double>();
            var inGrads = new List<Vector3d>();
            var inGeoms = new List<GeometryBase>();

            foreach (object obj in rawSource)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;
                if (val is double d) inScalars.Add(d);
                else if (val is int iv) inScalars.Add(iv);
                else if (val is Vector3d vec) inGrads.Add(vec);
                else if (val is Point3d pt) inGeoms.Add(new Rhino.Geometry.Point(pt));
                else if (val is GeometryBase gb) inGeoms.Add(gb);
            }

            double[] customDist = null;
            if (inScalars.Count > 0)
            {
                customDist = ddg.MapScalarInputToVertices(inScalars);
            }
            else if (inGeoms.Count > 0)
            {
                customDist = DdgAlgorithms.ComputeExternalGeometryHeatDistance(
                    ddg, inGeoms, 1.0, out _, out _, out _, out _);
            }

            List<Polyline> contactPaths = DdgAlgorithms.ComputeConcentricMillingToolpaths(
                ddg,
                customDist,
                null,
                stepover,
                toolR,
                out List<Polyline> toolCenterPaths,
                out _,
                out List<Plane> rawFrames,
                out _,
                out Mesh coloredM,
                out Mesh rollingOffsetGeo,
                inGrads.Count > 0 ? inGrads : null);

            // Smooth and collision-stabilize the 5/6-Axis tool frames so drastic turns are eliminated!
            List<Plane> smoothFrames = DdgAlgorithms.ComputeCollisionFreeSmoothPlanes(
                ddg, rawFrames, 2.5, Math.Max(ddg.MeanEdgeLength, Math.Abs(toolR)), out _, out _);

            _toolpaths.AddRange(toolCenterPaths);
            _frames.AddRange(smoothFrames);

            DA.SetDataList(0, contactPaths);
            DA.SetDataList(1, toolCenterPaths);
            DA.SetDataList(2, smoothFrames);
            DA.SetData(3, rollingOffsetGeo);
            DA.SetData(4, coloredM);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _toolpaths.Count == 0) return;

            Color pathCol = Attributes.Selected ? Color.FromArgb(255, 240, 60) : Color.FromArgb(0, 230, 180);
            foreach (var pl in _toolpaths)
                args.Display.DrawPolyline(pl, pathCol, 2);

            for (int i = 0; i < _frames.Count; i++)
            {
                Plane f = _frames[i];
                args.Display.DrawLine(f.Origin, f.Origin + _axisScale * f.XAxis, Color.FromArgb(240, 60, 60), 2);
                args.Display.DrawLine(f.Origin, f.Origin + _axisScale * f.YAxis, Color.FromArgb(60, 220, 90), 2);
                args.Display.DrawLine(f.Origin, f.Origin + _axisScale * 1.4 * f.ZAxis, Color.FromArgb(60, 150, 255), 2);
            }
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000039");
    }

    public class GH_IsoBoxSubtraction : GH_Component
    {
        private readonly List<Box> _boxes = new List<Box>();

        public GH_IsoBoxSubtraction()
          : base("3D Heat Offset & Vector Isosurface", "HeatIsoSurf",
              "Reconstructs a watertight 3D Offset Mesh, Smooth Boolean Union Isosurface, and Subtractive Iso-Boxes from multiple Meshes, Curves, or a 3D Vector Field (Signed Heat Method + Marching Tetrahedra).",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("IsoBoxSub");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGeometryParameter("Geometries", "G", "One or multiple Meshes, Curves, Breps, or Points to offset and unite into a smooth 3D Heat Isosurface", GH_ParamAccess.list);
            pManager.AddNumberParameter("Offset / Iso-Distance", "D", "Exact 3D offset / isosurface distance in real Rhino units (e.g. 2.5 or 4.55)", GH_ParamAccess.item, 2.5);
            int idxV = pManager.AddVectorParameter("3D Vector Field", "V", "Optional 3D vector field to modulate the Signed Heat Isosurface", GH_ParamAccess.list);
            pManager[idxV].Optional = true;
            pManager.AddIntegerParameter("Resolution", "R", "3D Marching-Tetrahedra voxel grid resolution (12..36)", GH_ParamAccess.item, 20);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Isosurface Mesh", "IM", "Watertight 3D Heat Offset / Smooth Boolean Union Isosurface Mesh", GH_ParamAccess.item);
            pManager.AddBoxParameter("Subtractive Iso-Boxes", "B", "3D voxel stock boxes inside the isosurface volume for CNC roughing / 3D print infill", GH_ParamAccess.list);
            pManager.AddNumberParameter("Field Values", "U", "Solved 3D Heat / Signed Distance scalar values", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _boxes.Clear();

            var geoms = new List<GeometryBase>();
            double isoDist = 2.5;
            var vecField = new List<Vector3d>();
            int res = 20;

            if (!DA.GetDataList(0, geoms) || geoms.Count == 0) return;
            DA.GetData(1, ref isoDist);
            DA.GetDataList(2, vecField);
            DA.GetData(3, ref res);

            Mesh isoMesh = DdgAlgorithms.Compute3DHeatIsosurfaceAndOffset(
                geoms,
                isoDist,
                vecField.Count > 0 ? vecField : null,
                res,
                out List<Box> boxes,
                out List<double> fieldVals);

            _boxes.AddRange(boxes);

            DA.SetData(0, isoMesh);
            DA.SetDataList(1, boxes);
            DA.SetDataList(2, fieldVals);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _boxes.Count == 0) return;

            Color boxCol = Color.FromArgb(90, 255, 165, 50);
            int stride = Math.Max(1, _boxes.Count / 180);
            for (int i = 0; i < _boxes.Count; i += stride)
                args.Display.DrawBox(_boxes[i], boxCol, 1);
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000040");
    }

    public class GH_GeodesicTSPRobot : GH_Component
    {
        private Polyline _tourPath;
        private readonly List<Plane> _robotFrames = new List<Plane>();
        private readonly List<Point3d> _collisionPts = new List<Point3d>();
        private double _frameSize = 1.0;

        public GH_GeodesicTSPRobot()
          : base("Geodesic TSP & Robot Plane Smooth", "RobotPlanes",
              "Computes shortest Geodesic TSP trajectories on a mesh AND smooths/stabilizes 6-Axis Robot or CNC Planes in SO(3) to eliminate drastic turns and avoid mesh concavities/collisions.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GeoTSP");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input 3D surface mesh", GH_ParamAccess.item);
            int idxP = pManager.AddGenericParameter("Planes or Points", "P", "Optional list of Planes (to smooth & collision-stabilize) or Points (to visit via Geodesic TSP). Leave empty to visit mesh vertices automatically!", GH_ParamAccess.list);
            pManager[idxP].Optional = true;
            pManager.AddNumberParameter("Plane Smoothness", "S", "SO(3) frame rotation smoothing & collision-tilt strength (eliminates drastic turns & wrist flips)", GH_ParamAccess.item, 3.0);
            pManager.AddNumberParameter("Tool Clearance / Offset", "R", "Tool clearance radius & normal standoff offset", GH_ParamAccess.item, 1.0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Robot Trajectory", "Path", "Continuous geodesic tour or toolpath polyline", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Smooth 6-Axis Planes", "TCP", "Collision-avoiding, twist-free smoothed 6-Axis Robot Planes", GH_ParamAccess.list);
            pManager.AddPointParameter("Collision-Tilted Points", "CP", "Locations where tool orientation was automatically tilted to avoid mesh collision", GH_ParamAccess.list);
            pManager.AddNumberParameter("Total Length", "L", "Total physical trajectory length", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _tourPath = null;
            _robotFrames.Clear();
            _collisionPts.Clear();

            Mesh m = null;
            var rawItems = new List<object>();
            double smooth = 3.0;
            double clearance = 1.0;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetDataList(1, rawItems);
            DA.GetData(2, ref smooth);
            DA.GetData(3, ref clearance);

            var ddg = new DdgMesh(m);
            _frameSize = ddg.MeanEdgeLength * 0.85;

            var inPlanes = new List<Plane>();
            var inPoints = new List<Point3d>();

            foreach (object obj in rawItems)
            {
                if (obj == null) continue;
                if (obj is GH_Plane ghp) inPlanes.Add(ghp.Value);
                else if (obj is Plane pl) inPlanes.Add(pl);
                else if (obj is GH_Point ghpt) inPoints.Add(ghpt.Value);
                else if (obj is Point3d pt) inPoints.Add(pt);
            }

            List<Plane> baseFrames;
            double totalLen = 0.0;

            if (inPlanes.Count >= 2)
            {
                // User supplied existing Planes -> smooth drastic turns & tilt away from collisions!
                baseFrames = inPlanes;
                var pts = new List<Point3d>(inPlanes.Count);
                for (int i = 0; i < inPlanes.Count; i++)
                {
                    pts.Add(inPlanes[i].Origin);
                    if (i > 0) totalLen += inPlanes[i].Origin.DistanceTo(inPlanes[i - 1].Origin);
                }
                _tourPath = new Polyline(pts);
            }
            else
            {
                // Run Heat Geodesic TSP across target points or mesh vertices
                _tourPath = DdgAlgorithms.ComputeGeodesicTSPAndRobotFrames(
                    ddg,
                    inPoints.Count >= 2 ? inPoints : null,
                    80,
                    true,
                    clearance,
                    true,
                    out baseFrames,
                    out _,
                    out totalLen);
            }

            List<Plane> smoothFrames = DdgAlgorithms.ComputeCollisionFreeSmoothPlanes(
                ddg, baseFrames, smooth, Math.Max(ddg.MeanEdgeLength, Math.Abs(clearance)),
                out List<Point3d> colPts, out _);

            _robotFrames.AddRange(smoothFrames);
            _collisionPts.AddRange(colPts);

            DA.SetData(0, _tourPath);
            DA.SetDataList(1, smoothFrames);
            DA.SetDataList(2, colPts);
            DA.SetData(3, totalLen);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _tourPath == null) return;

            Color cTour = Attributes.Selected ? Color.FromArgb(255, 240, 60) : Color.FromArgb(0, 240, 200);
            args.Display.DrawPolyline(_tourPath, cTour, 3);

            int stride = Math.Max(1, _robotFrames.Count / 60);
            for (int i = 0; i < _robotFrames.Count; i += stride)
            {
                Plane pl = _robotFrames[i];
                args.Display.DrawLine(pl.Origin, pl.Origin + _frameSize * pl.XAxis, Color.FromArgb(245, 65, 65), 2);
                args.Display.DrawLine(pl.Origin, pl.Origin + _frameSize * pl.YAxis, Color.FromArgb(65, 225, 95), 2);
                args.Display.DrawLine(pl.Origin, pl.Origin + _frameSize * 1.35 * pl.ZAxis, Color.FromArgb(65, 160, 255), 2);
            }

            if (_collisionPts.Count > 0)
                args.Display.DrawPoints(_collisionPts, Rhino.Display.PointStyle.RoundSimple, 4, Color.Gold);
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000041");
    }

    public class GH_MeshMedialSkeleton : GH_Component
    {
        private readonly List<Polyline> _skelCurves = new List<Polyline>();
        private readonly List<Point3d> _medialNodes = new List<Point3d>();

        public GH_MeshMedialSkeleton()
          : base("Mesh Medial Skeleton & Spine", "Skeleton",
              "Extracts the 3D Medial Axis Skeleton Curves and Shrinking-Ball Spine of a mesh via Implicit Mean Curvature Contraction and Tangent-Point Inscribed Spheres.",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("WalkOnMesh");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Input 3D closed or open mesh", GH_ParamAccess.item);
            pManager.AddNumberParameter("Contraction", "C", "Laplacian medial contraction strength (1..10)", GH_ParamAccess.item, 2.5);
            pManager.AddIntegerParameter("Branches", "N", "Skeleton branch node density", GH_ParamAccess.item, 12);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Skeleton Curves", "SK", "Extracted 3D medial skeleton graph polylines", GH_ParamAccess.list);
            pManager.AddMeshParameter("Contracted Spine", "SM", "Contracted medial spine mesh", GH_ParamAccess.item);
            pManager.AddPointParameter("Medial Nodes", "MP", "3D medial axis nodes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Inscribed Radii", "R", "Local inscribed sphere / wall thickness radius per vertex", GH_ParamAccess.list);
            pManager.AddMeshParameter("Thickness Mesh", "CM", "Mesh colored by local medial thickness", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _skelCurves.Clear();
            _medialNodes.Clear();

            Mesh m = null;
            double contract = 2.5;
            int branches = 12;

            if (!DA.GetData(0, ref m) || m == null) return;
            DA.GetData(1, ref contract);
            DA.GetData(2, ref branches);

            var ddg = new DdgMesh(m);
            List<Polyline> skel = DdgAlgorithms.ComputeMeshMedialSkeleton(
                ddg, contract, branches,
                out Mesh spineMesh,
                out List<Point3d> nodes,
                out double[] radii,
                out Mesh coloredM);

            _skelCurves.AddRange(skel);
            _medialNodes.AddRange(nodes);

            DA.SetDataList(0, skel);
            DA.SetData(1, spineMesh);
            DA.SetDataList(2, nodes);
            DA.SetDataList(3, radii);
            DA.SetData(4, coloredM);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked || _skelCurves.Count == 0) return;

            Color cSkel = Attributes.Selected ? Color.FromArgb(255, 255, 80) : Color.FromArgb(255, 95, 215);
            foreach (var pl in _skelCurves)
                args.Display.DrawPolyline(pl, cSkel, 3);

            if (_medialNodes.Count > 0)
                args.Display.DrawPoints(_medialNodes, Rhino.Display.PointStyle.RoundControlPoint, 4, Color.Cyan);
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000042");
    }

    public class GH_RepulsiveShellsRobotPath : GH_Component
    {
        private Polyline _robotPath3D;
        private Polyline _cSpaceGeodesic2D;
        private Mesh _cSpaceMapMesh;
        private Mesh _spindleShellMesh;
        private readonly List<Plane> _robotPlanes = new List<Plane>();
        private double _axisDrawSize = 2.0;

        public GH_RepulsiveShellsRobotPath()
          : base("Repulsive Shells & C-Space Robot Path", "RepulsiveBot",
              "Computes optimal collision- & singularity-free Robot Toolpaths from A to B and builds the 2D Configuration-Space (C-Space) Geodesic Map (Sassen, Schumacher, Rumpf, Wardetzky, Crane 'Repulsive Shells').",
              "CraneGH", "05. Mold, Segmentation & CAM")
        {
        }

        protected override Bitmap Icon => CraneIcons.GetIcon("GeodesicPath");

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Path / A-B", "P", "Toolpath Curve/Polyline, list of Planes, or [Start A, End B] Points/Planes for Pick & Place", GH_ParamAccess.list);
            pManager.AddMeshParameter("Obstacles", "Obs", "Obstacle meshes (printed part, fixtures, table, robot body)", GH_ParamAccess.list);
            pManager.AddNumberParameter("Spindle Radius", "R", "Spindle / extruder shell radius (0 = auto-scaled)", GH_ParamAccess.item, 0.0);
            pManager.AddIntegerParameter("Mode", "Mode", "0 = Fixed TCP + Spindle Orientation Geodesic (3D Print/Mill), 1 = Pick & Place 3D Obstacle Avoidance (A to B), 2 = Spherical Cone Map [Theta, Phi]", GH_ParamAccess.item, 0);
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Robot Path", "Path", "Optimal 3D Robot TCP trajectory from A to B", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Robot Planes", "Pl", "Collision- & singularity-free Robot TCP Planes along the trajectory", GH_ParamAccess.list);
            pManager.AddMeshParameter("2D C-Space Map", "CMap", "2D Configuration-Space Energy Landscape Mesh (Blue = safe valley, Red = spindle collision / wrist singularity peak)", GH_ParamAccess.item);
            pManager.AddCurveParameter("2D C-Space Geodesic", "CPath", "Optimal Geodesic Path from A to B traced on the 2D C-Space Map", GH_ParamAccess.item);
            pManager.AddMeshParameter("Spindle Shell", "Shell", "3D swept Spindle & Tool-Holder Shell preview mesh verifying obstacle clearance", GH_ParamAccess.item);
            pManager.AddNumberParameter("Clearance", "Clr", "Minimum spindle-to-obstacle clearance distance at each step", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            _robotPath3D = null;
            _cSpaceGeodesic2D = null;
            _cSpaceMapMesh = null;
            _spindleShellMesh = null;
            _robotPlanes.Clear();

            var rawInputs = new List<object>();
            var obstacles = new List<Mesh>();
            double radius = 0.0;
            int mode = 0;

            if (!DA.GetDataList(0, rawInputs) || rawInputs.Count == 0) return;
            DA.GetDataList(1, obstacles);
            DA.GetData(2, ref radius);
            DA.GetData(3, ref mode);

            // Parse input Path / Planes / Points into initial reference frames
            var inputFrames = new List<Plane>();
            var inputPoints = new List<Point3d>();

            foreach (object obj in rawInputs)
            {
                if (obj == null) continue;
                object val = (obj is IGH_Goo goo) ? goo.ScriptVariable() : obj;

                if (val is Plane pl)
                {
                    inputFrames.Add(pl);
                }
                else if (val is Point3d pt)
                {
                    inputPoints.Add(pt);
                }
                else if (val is Curve crv && crv.IsValid)
                {
                    int nDiv = 40;
                    double[] tParams = crv.DivideByCount(nDiv, true);
                    if (tParams != null)
                    {
                        foreach (double t in tParams)
                        {
                            Point3d p = crv.PointAt(t);
                            Vector3d tang = crv.TangentAt(t);
                            if (tang.Length < 1e-10) tang = Vector3d.XAxis;
                            tang.Unitize();
                            Vector3d up = Vector3d.ZAxis;
                            // Estimate normal away from nearest obstacle if available
                            if (obstacles.Count > 0 && obstacles[0] != null)
                            {
                                MeshPoint mp = obstacles[0].ClosestMeshPoint(p, 0.0);
                                if (mp != null)
                                {
                                    Vector3d nrm = obstacles[0].NormalAt(mp);
                                    if (nrm.Length > 1e-10 && nrm.Z >= -0.2) up = nrm;
                                }
                            }
                            Vector3d yAx = Vector3d.CrossProduct(up, tang);
                            if (yAx.Length < 1e-10)
                                yAx = Vector3d.CrossProduct(up, Math.Abs(up.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                            yAx.Unitize();
                            Vector3d xAx = Vector3d.CrossProduct(yAx, up);
                            xAx.Unitize();
                            inputFrames.Add(new Plane(p, xAx, yAx));
                        }
                    }
                }
                else if (val is Polyline poly && poly.Count >= 2)
                {
                    for (int i = 0; i < poly.Count; i++)
                    {
                        Vector3d tang = (i < poly.Count - 1) ? (poly[i + 1] - poly[i]) : (poly[i] - poly[i - 1]);
                        if (tang.Length < 1e-10) tang = Vector3d.XAxis;
                        tang.Unitize();
                        Vector3d yAx = Vector3d.CrossProduct(Vector3d.ZAxis, tang);
                        if (yAx.Length < 1e-10) yAx = Vector3d.YAxis;
                        yAx.Unitize();
                        Vector3d xAx = Vector3d.CrossProduct(yAx, Vector3d.ZAxis);
                        xAx.Unitize();
                        inputFrames.Add(new Plane(poly[i], xAx, yAx));
                    }
                }
            }

            if (inputFrames.Count == 0 && inputPoints.Count >= 2)
            {
                // User passed Pick & Place points [A, B, ...] -> default to Pick & Place Mode 1 if mode wasn't explicitly set
                for (int i = 0; i < inputPoints.Count; i++)
                {
                    Vector3d tang = (i < inputPoints.Count - 1)
                        ? (inputPoints[i + 1] - inputPoints[i])
                        : (inputPoints[i] - inputPoints[i - 1]);
                    if (tang.Length < 1e-10) tang = Vector3d.XAxis;
                    tang.Unitize();
                    Vector3d yAx = Vector3d.CrossProduct(Vector3d.ZAxis, tang);
                    if (yAx.Length < 1e-10) yAx = Vector3d.YAxis;
                    yAx.Unitize();
                    Vector3d xAx = Vector3d.CrossProduct(yAx, Vector3d.ZAxis);
                    xAx.Unitize();
                    inputFrames.Add(new Plane(inputPoints[i], xAx, yAx));
                }
                if (inputPoints.Count == 2 && Params.Input[3].SourceCount == 0)
                    mode = 1; // Auto-select Pick & Place 3D trajectory when given just point A and point B
            }
            else if (inputFrames.Count == 2 && Params.Input[3].SourceCount == 0)
            {
                mode = 1; // Auto-select Pick & Place 3D trajectory when given just Plane A and Plane B
            }

            if (inputFrames.Count == 0) return;

            _robotPath3D = DdgAlgorithms.ComputeRepulsiveShellsAndCSpaceRobotPath(
                inputFrames,
                obstacles,
                radius,
                mode,
                out List<Plane> optPlanes,
                out _cSpaceMapMesh,
                out _cSpaceGeodesic2D,
                out _spindleShellMesh,
                out double[] clearances);

            _robotPlanes.AddRange(optPlanes);
            if (_robotPath3D != null && _robotPath3D.Count >= 2)
                _axisDrawSize = Math.Max(0.5, _robotPath3D.BoundingBox.Diagonal.Length * 0.045);

            DA.SetData(0, _robotPath3D);
            DA.SetDataList(1, optPlanes);
            DA.SetData(2, _cSpaceMapMesh);
            DA.SetData(3, _cSpaceGeodesic2D);
            DA.SetData(4, _spindleShellMesh);
            DA.SetDataList(5, clearances);
        }

        public override void DrawViewportMeshes(IGH_PreviewArgs args)
        {
            base.DrawViewportMeshes(args);
            if (Hidden || Locked) return;

            if (_cSpaceMapMesh != null && _cSpaceMapMesh.Vertices.Count > 0)
                args.Display.DrawMeshFalseColors(_cSpaceMapMesh);

            if (_spindleShellMesh != null && _spindleShellMesh.Vertices.Count > 0)
                args.Display.DrawMeshFalseColors(_spindleShellMesh);
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);
            if (Hidden || Locked) return;

            if (_robotPath3D != null && _robotPath3D.Count >= 2)
                args.Display.DrawPolyline(_robotPath3D, Color.FromArgb(0, 255, 210), 3);

            if (_cSpaceGeodesic2D != null && _cSpaceGeodesic2D.Count >= 2)
            {
                args.Display.DrawPolyline(_cSpaceGeodesic2D, Color.FromArgb(255, 255, 255), 4);
                args.Display.DrawPoint(_cSpaceGeodesic2D[0], Rhino.Display.PointStyle.RoundControlPoint, 6, Color.Lime);
                args.Display.DrawPoint(_cSpaceGeodesic2D[_cSpaceGeodesic2D.Count - 1], Rhino.Display.PointStyle.RoundControlPoint, 6, Color.Aqua);
            }

            int stride = Math.Max(1, _robotPlanes.Count / 22);
            for (int i = 0; i < _robotPlanes.Count; i += stride)
            {
                Plane pl = _robotPlanes[i];
                args.Display.DrawLine(pl.Origin, pl.Origin + _axisDrawSize * pl.XAxis, Color.FromArgb(245, 70, 70), 2);
                args.Display.DrawLine(pl.Origin, pl.Origin + _axisDrawSize * pl.YAxis, Color.FromArgb(70, 230, 95), 2);
                args.Display.DrawLine(pl.Origin, pl.Origin + _axisDrawSize * 1.4 * pl.ZAxis, Color.FromArgb(75, 165, 255), 2);
            }
        }

        public override Guid ComponentGuid => new Guid("A109B209-C309-4D09-9E09-000000000043");
    }
}
