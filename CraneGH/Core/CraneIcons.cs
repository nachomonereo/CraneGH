using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace CraneGH.Core
{
    /// <summary>
    /// Procedural 24x24 anti-aliased icon renderer with a unique, high-contrast visual logo
    /// for every single component in CraneGH.
    /// </summary>
    public static class CraneIcons
    {
        private static readonly Dictionary<string, Bitmap> _cache = new Dictionary<string, Bitmap>();

        public static Bitmap GetIcon(string key)
        {
            if (_cache.TryGetValue(key, out Bitmap bmp))
                return bmp;

            bmp = RenderIcon(key, 24);
            _cache[key] = bmp;
            return bmp;
        }

        public static Bitmap RenderIcon(string key, int size = 24)
        {
            Bitmap bmp = new Bitmap(size, size);
            using (Graphics g = Graphics.FromImage(bmp))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.Clear(Color.Transparent);

                float s = size / 24f;
                g.ScaleTransform(s, s);

                switch (key)
                {
                    // 01. Visualizers
                    case "ScalarView": DrawScalarView(g); break;
                    case "GradView": DrawGradView(g); break;
                    case "VecView": DrawVecView(g); break;
                    case "CrossView": DrawCrossView(g); break;

                    // 02. Scalar & Geodesics
                    case "Curvatures": DrawCurvatures(g); break;
                    case "SmoothScalar": DrawSmoothScalar(g); break;
                    case "HeatDist": DrawHeatDist(g); break;
                    case "IsoCut": DrawIsoCut(g); break;
                    case "GeoPath": DrawGeoPath(g); break;
                    case "FlipOut": DrawFlipOut(g); break;

                    // 03. Vector Fields & Stripes
                    case "RotateVec": DrawRotateVec(g); break;
                    case "SmoothField": DrawSmoothField(g); break;
                    case "CrossField": DrawCrossField(g); break;
                    case "VecHeat": DrawVecHeat(g); break;
                    case "WalkOnMesh": DrawWalkOnMesh(g); break;
                    case "GuidedFlow": DrawGuidedFlow(g); break;
                    case "Stripes": DrawStripes(g); break;
                    case "VarStripes": DrawVarStripes(g); break;

                    // 04. Segmentation & Fabrication
                    case "KMeansSeg": DrawKMeansSeg(g); break;
                    case "CurvSeg": DrawCurvSeg(g); break;
                    case "ScalarSeg": DrawScalarSeg(g); break;
                    case "VecFamily": DrawVecFamily(g); break;
                    case "MoldParting": DrawMoldParting(g); break;
                    case "MillingPaths": DrawMillingPaths(g); break;
                    case "RollSphere": DrawRollSphere(g); break;
                    case "IsoBoxSub": DrawIsoBoxSub(g); break;
                    case "Skeleton": DrawSkeleton(g); break;
                    case "CleanCloud": DrawCleanCloud(g); break;
                    case "MCFlow": DrawMCFlow(g); break;

                    // Additional active & legacy keys
                    case "OptField": DrawOptField(g); break;
                    case "CrvStripes": DrawCrvStripes(g); break;
                    case "CrossQuads": DrawCrossQuads(g); break;
                    case "MeshSeg": DrawMeshSeg(g); break;
                    case "GradLap": DrawGradView(g); break;
                    case "Harmonic": DrawScalarView(g); break;
                    case "IsoCrvs": DrawIsoCut(g); break;
                    case "LogMap": DrawCrossView(g); break;
                    case "DecalMap": DrawCrossView(g); break;
                    case "TrivConn": DrawCrossView(g); break;
                    case "Streamlines": DrawGuidedFlow(g); break;
                    case "SpinXForm": DrawCurvatures(g); break;
                    case "BFF": DrawScalarView(g); break;
                    case "Auxetics": DrawCrossField(g); break;
                    case "RepCurve": DrawFlipOut(g); break;
                    case "WoS": DrawCleanCloud(g); break;
                    case "SubgridTet": DrawIsoBoxSub(g); break;
                    case "SharpEdges": DrawKMeansSeg(g); break;
                    case "FeatHeat": DrawHeatDist(g); break;
                    case "VarCuts": DrawCurvSeg(g); break;
                    case "GeoTSP": DrawMillingPaths(g); break;
                    default: DrawHeatDist(g); break;
                }
            }
            return bmp;
        }

        private static void DrawDiscBackground(Graphics g, Color c1, Color c2)
        {
            using (var brush = new LinearGradientBrush(new RectangleF(1, 1, 22, 22), c1, c2, 45f))
            {
                g.FillEllipse(brush, 1.5f, 1.5f, 21f, 21f);
            }
            using (var pen = new Pen(Color.FromArgb(200, 20, 24, 30), 1.1f))
            {
                g.DrawEllipse(pen, 1.5f, 1.5f, 21f, 21f);
            }
        }

        // =========================================================================
        // 01. VISUALIZERS
        // =========================================================================

        // ScalarView: Heatmap disc + white iso-arcs + vertical color bar
        private static void DrawScalarView(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(25, 55, 145), Color.FromArgb(245, 85, 35));
            using (var isoPen = new Pen(Color.FromArgb(230, 255, 245, 190), 1.3f))
            {
                g.DrawArc(isoPen, 4, 4, 15, 15, 195, 145);
                g.DrawArc(isoPen, 6.5f, 6.5f, 11, 11, 195, 145);
            }
            using (var barBrush = new LinearGradientBrush(new RectangleF(17.5f, 4f, 3.5f, 16f), Color.Yellow, Color.Crimson, 90f))
            {
                g.FillRectangle(barBrush, 17.5f, 4f, 3.5f, 16f);
                g.DrawRectangle(Pens.White, 17, 4, 4, 16);
            }
        }

        // GradView: Warm slope disc + cyan gradient segments + white orthogonal wavefront ticks
        private static void DrawGradView(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(145, 25, 45), Color.FromArgb(30, 45, 85));
            using (var wfPen = new Pen(Color.FromArgb(190, 255, 230, 140), 1.2f))
            {
                g.DrawArc(wfPen, 4, 4, 16, 16, 205, 130);
                g.DrawArc(wfPen, 7, 7, 12, 12, 205, 130);
            }
            using (var gradPen = new Pen(Color.FromArgb(0, 245, 255), 1.9f))
            {
                g.DrawLine(gradPen, 6, 17, 11, 11);
                g.DrawLine(gradPen, 11, 18, 16, 12);
                g.DrawLine(gradPen, 9, 12, 15, 5);
            }
            g.FillEllipse(Brushes.White, 10, 10, 2.5f, 2.5f);
            g.FillEllipse(Brushes.White, 15, 11, 2.5f, 2.5f);
            g.FillEllipse(Brushes.White, 14, 4, 2.5f, 2.5f);
        }

        // VecView: Dark mesh triangle + bright gold/cyan directional line segments
        private static void DrawVecView(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(28, 36, 52), Color.FromArgb(16, 20, 30));
            using (var gridPen = new Pen(Color.FromArgb(90, 130, 160, 200), 1.0f))
            {
                g.DrawLine(gridPen, 4, 18, 12, 5);
                g.DrawLine(gridPen, 12, 5, 20, 18);
                g.DrawLine(gridPen, 4, 18, 20, 18);
            }
            using (var segPen = new Pen(Color.FromArgb(255, 220, 55), 2.0f))
            {
                g.DrawLine(segPen, 5, 16, 11, 11);
                g.DrawLine(segPen, 12, 16, 18, 11);
                g.DrawLine(segPen, 9, 9, 16, 5);
            }
            g.FillEllipse(Brushes.Cyan, 10, 10, 2.6f, 2.6f);
            g.FillEllipse(Brushes.Cyan, 17, 10, 2.6f, 2.6f);
            g.FillEllipse(Brushes.Cyan, 15, 4, 2.6f, 2.6f);
        }

        // CrossView: X-Crosses + Divergence/Rotor ring
        private static void DrawCrossView(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(38, 28, 68), Color.FromArgb(20, 16, 38));
            using (var ringPen = new Pen(Color.FromArgb(140, 255, 190, 60), 1.2f) { DashStyle = DashStyle.Dot })
            {
                g.DrawEllipse(ringPen, 4.5f, 4.5f, 15f, 15f);
            }
            using (var pU = new Pen(Color.FromArgb(0, 235, 255), 1.8f))
            using (var pV = new Pen(Color.FromArgb(255, 95, 85), 1.8f))
            {
                PointF[] pts = { new PointF(8, 8), new PointF(16, 8), new PointF(12, 12), new PointF(8, 16), new PointF(16, 16) };
                foreach (var c in pts)
                {
                    g.DrawLine(pU, c.X - 2.3f, c.Y - 2.3f, c.X + 2.3f, c.Y + 2.3f);
                    g.DrawLine(pV, c.X - 2.3f, c.Y + 2.3f, c.X + 2.3f, c.Y - 2.3f);
                }
            }
        }

        // =========================================================================
        // 02. SCALAR & GEODESICS
        // =========================================================================

        // Curvatures: Saddle curvature arcs + principal axes
        private static void DrawCurvatures(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(35, 105, 205), Color.FromArgb(225, 55, 50));
            using (var pen = new Pen(Color.FromArgb(240, 255, 255, 255), 1.5f))
            {
                g.DrawArc(pen, 4f, 8f, 16f, 8f, 0, 180);
                g.DrawArc(pen, 8f, 4f, 8f, 16f, 90, 180);
            }
            using (var p2 = new Pen(Color.Gold, 1.6f))
            {
                g.DrawLine(p2, 12, 3, 12, 21);
                g.DrawLine(p2, 3, 12, 21, 12);
            }
        }

        // SmoothScalar: Noisy red wave -> Smooth glowing cyan bell curve
        private static void DrawSmoothScalar(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(42, 52, 82), Color.FromArgb(22, 28, 48));
            using (var noisyPen = new Pen(Color.FromArgb(140, 255, 105, 85), 1.1f))
            {
                PointF[] jagged = {
                    new PointF(3, 16), new PointF(5.5f, 7), new PointF(8, 17),
                    new PointF(10.5f, 5), new PointF(13, 14), new PointF(15.5f, 5),
                    new PointF(18, 16), new PointF(21, 13)
                };
                g.DrawLines(noisyPen, jagged);
            }
            using (var smoothPen = new Pen(Color.FromArgb(80, 250, 215), 2.4f))
            {
                g.DrawBezier(smoothPen, new PointF(3, 16), new PointF(9, 3), new PointF(15, 3), new PointF(21, 16));
            }
        }

        // HeatDist: Keenan Crane's Crimson-Orange-Yellow-White Heatmap
        private static void DrawHeatDist(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(125, 15, 22), Color.FromArgb(215, 45, 30));
            g.FillEllipse(new SolidBrush(Color.FromArgb(245, 120, 30)), 3f, 3f, 15f, 15f);
            g.FillEllipse(new SolidBrush(Color.FromArgb(255, 205, 60)), 5.2f, 5.2f, 10.2f, 10.2f);
            g.FillEllipse(Brushes.White, 7.6f, 7.6f, 5f, 5f);

            using (var pen = new Pen(Color.FromArgb(215, 255, 245, 200), 1.0f))
            {
                g.DrawEllipse(pen, 5.2f, 5.2f, 10.2f, 10.2f);
                g.DrawEllipse(pen, 3f, 3f, 15f, 15f);
            }
            g.FillEllipse(Brushes.Red, 9f, 9f, 2.5f, 2.5f);
        }

        // IsoCut: Clean glowing gold iso-splitting curves on dark plum mesh
        private static void DrawIsoCut(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(55, 28, 48), Color.FromArgb(28, 16, 32));
            using (var pen = new Pen(Color.FromArgb(255, 220, 55), 1.8f))
            {
                g.DrawArc(pen, 3f, 3f, 18f, 18f, 200, 140);
                g.DrawArc(pen, 5.5f, 6f, 13f, 13f, 200, 140);
                g.DrawArc(pen, 8f, 9f, 8f, 8f, 200, 140);
            }
        }

        // GeoPath: Heatmap + bright cyan shortest path A-to-B
        private static void DrawGeoPath(Graphics g)
        {
            DrawHeatDist(g);
            using (var pen = new Pen(Color.FromArgb(0, 255, 220), 2.5f))
            {
                g.DrawBezier(pen, new PointF(10, 10), new PointF(14, 8), new PointF(15, 16), new PointF(19, 18));
            }
            g.FillEllipse(Brushes.Cyan, 17.5f, 16.5f, 3.6f, 3.6f);
        }

        // FlipOut: Blue mesh + gold edges + taut red geodesic curve
        private static void DrawFlipOut(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(0, 135, 230), Color.FromArgb(0, 85, 170));
            using (var goldPen = new Pen(Color.FromArgb(255, 195, 40), 1.1f))
            {
                g.DrawLine(goldPen, 3, 12, 12, 3);
                g.DrawLine(goldPen, 12, 3, 21, 12);
                g.DrawLine(goldPen, 21, 12, 12, 21);
                g.DrawLine(goldPen, 12, 21, 3, 12);
                g.DrawLine(goldPen, 3, 12, 21, 12);
            }
            using (var redPen = new Pen(Color.FromArgb(250, 35, 40), 2.5f))
            {
                g.DrawLine(redPen, 5, 18, 19, 6);
            }
            g.FillEllipse(Brushes.White, 3.5f, 16.5f, 3.5f, 3.5f);
            g.FillEllipse(Brushes.White, 17f, 4.5f, 3.5f, 3.5f);
        }

        // =========================================================================
        // 03. VECTOR FIELDS & STRIPES
        // =========================================================================

        // RotateVec: Tangent vector rotated by angle A around normal (faint cyan -> curved arc -> bright gold)
        private static void DrawRotateVec(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(22, 62, 72), Color.FromArgb(14, 36, 45));
            // Original vector (faint cyan horizontal)
            using (var origPen = new Pen(Color.FromArgb(140, 100, 225, 255), 1.8f))
            {
                g.DrawLine(origPen, 6, 17, 19, 17);
            }
            // Rotation arc
            using (var arcPen = new Pen(Color.FromArgb(235, 255, 255, 255), 1.4f) { DashStyle = DashStyle.Dot })
            {
                g.DrawArc(arcPen, 8, 9, 16, 16, 270, 90);
            }
            // Rotated vector (bright gold vertical-diagonal)
            using (var rotPen = new Pen(Color.FromArgb(255, 220, 45), 2.4f))
            {
                DrawArrow(g, rotPen, 6, 17, 10, 5);
            }
            g.FillEllipse(Brushes.White, 4.2f, 15.2f, 3.6f, 3.6f);
        }

        // SmoothField: Flowing aligned tangent segments
        private static void DrawSmoothField(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(28, 62, 98), Color.FromArgb(16, 34, 56));
            using (var flowPen = new Pen(Color.FromArgb(110, 255, 220, 100), 1.2f))
            {
                g.DrawBezier(flowPen, new PointF(3, 16), new PointF(9, 10), new PointF(15, 10), new PointF(21, 6));
                g.DrawBezier(flowPen, new PointF(3, 19), new PointF(9, 13), new PointF(15, 13), new PointF(21, 9));
            }
            using (var segPen = new Pen(Color.FromArgb(90, 245, 255), 2.0f))
            {
                g.DrawLine(segPen, 4.5f, 15.5f, 9.5f, 11.5f);
                g.DrawLine(segPen, 9.5f, 14.5f, 15.0f, 11.0f);
                g.DrawLine(segPen, 14.5f, 12.5f, 19.8f, 8.0f);
            }
        }

        // CrossField: Orthogonal U (cyan) and V (coral) crosses + gold singularity
        private static void DrawCrossField(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(28, 48, 82), Color.FromArgb(16, 28, 50));
            using (var pU = new Pen(Color.FromArgb(80, 230, 255), 1.7f))
            using (var pV = new Pen(Color.FromArgb(255, 110, 85), 1.7f))
            {
                PointF[] centers = { new PointF(7, 7), new PointF(17, 7), new PointF(12, 12), new PointF(7, 17), new PointF(17, 17) };
                foreach (var c in centers)
                {
                    g.DrawLine(pU, c.X - 2.6f, c.Y - 0.7f, c.X + 2.6f, c.Y + 0.7f);
                    g.DrawLine(pV, c.X - 0.7f, c.Y + 2.6f, c.X + 0.7f, c.Y - 2.6f);
                }
            }
            g.FillEllipse(Brushes.Gold, 10.5f, 10.5f, 3f, 3f);
        }

        // VecHeat: Vector Parallel Transport
        private static void DrawVecHeat(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(35, 95, 165), Color.FromArgb(20, 50, 95));
            using (var pen = new Pen(Color.FromArgb(160, 230, 255), 1.7f))
            {
                g.DrawLine(pen, 6, 17, 11, 9);
                g.DrawLine(pen, 11, 18, 16, 10);
                g.DrawLine(pen, 15, 16, 20, 8);
            }
            using (var srcPen = new Pen(Color.FromArgb(255, 205, 45), 2.3f))
            {
                DrawArrow(g, srcPen, 4, 13, 9, 4);
            }
        }

        // WalkOnMesh: Stepped trajectory from green start to crimson peak
        private static void DrawWalkOnMesh(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 75, 115), Color.FromArgb(20, 38, 65));
            PointF[] walk = { new PointF(4, 18), new PointF(8, 12), new PointF(13, 14), new PointF(16, 7), new PointF(20, 5) };
            using (var walkPen = new Pen(Color.FromArgb(255, 215, 50), 2.1f))
            {
                g.DrawLines(walkPen, walk);
            }
            foreach (var p in walk)
                g.FillEllipse(Brushes.White, p.X - 1.6f, p.Y - 1.6f, 3.2f, 3.2f);
            g.FillEllipse(Brushes.LimeGreen, walk[0].X - 2.3f, walk[0].Y - 2.3f, 4.6f, 4.6f);
            g.FillEllipse(Brushes.Crimson, walk[4].X - 2.3f, walk[4].Y - 2.3f, 4.6f, 4.6f);
        }

        // GuidedFlow: Blue avoid zone (0) + Gold attract highway (1) + Cyan guided path
        private static void DrawGuidedFlow(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(25, 45, 95), Color.FromArgb(215, 145, 30));
            using (var pathPen = new Pen(Color.FromArgb(0, 255, 220), 2.3f))
            {
                g.DrawBezier(pathPen, new PointF(4, 18), new PointF(11, 17), new PointF(13, 7), new PointF(20, 6));
            }
            using (var subPen = new Pen(Color.FromArgb(160, 255, 255, 255), 1.3f))
            {
                g.DrawBezier(subPen, new PointF(4, 13), new PointF(10, 13), new PointF(12, 5), new PointF(18, 4));
            }
            g.FillEllipse(Brushes.Gold, 17.5f, 4f, 3.5f, 3.5f);
        }

        // Stripes: Equidistant Zebra Stripes with Y-Bifurcation
        private static void DrawStripes(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(245, 245, 248), Color.FromArgb(210, 214, 222));
            using (var blackPen = new Pen(Color.FromArgb(25, 28, 35), 2.2f))
            {
                g.DrawLine(blackPen, 4, 5, 20, 5);
                g.DrawLine(blackPen, 3, 12, 11, 12);
                g.DrawLine(blackPen, 11, 12, 19, 8.5f);
                g.DrawLine(blackPen, 11, 12, 19, 15.5f);
                g.DrawLine(blackPen, 4, 19, 20, 19);
            }
            g.FillEllipse(new SolidBrush(Color.FromArgb(235, 50, 40)), 9.5f, 10.5f, 4f, 4f);
        }

        // VarStripes: Variable-Spacing Stripes (sparse on left where scalar is low, dense on right where scalar is high!)
        private static void DrawVarStripes(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 70, 135), Color.FromArgb(235, 155, 45));
            using (var stripePen = new Pen(Color.White, 1.7f))
            {
                // Sparse on left (low scalar)
                g.DrawLine(stripePen, 5.5f, 5f, 5.5f, 19f);
                g.DrawLine(stripePen, 10.5f, 3.5f, 10.5f, 20.5f);
                // Dense on right (high scalar / gradient)
                g.DrawLine(stripePen, 14.2f, 3.5f, 14.2f, 20.5f);
                g.DrawLine(stripePen, 16.8f, 4.2f, 16.8f, 19.8f);
                g.DrawLine(stripePen, 19.2f, 6.0f, 19.2f, 18.0f);
            }
        }

        // =========================================================================
        // 04. SEGMENTATION & FABRICATION
        // =========================================================================

        // KMeansSeg: Multi-colored clusters + geodesic seams + normal arrows
        private static void DrawKMeansSeg(Graphics g)
        {
            using (var b1 = new SolidBrush(Color.FromArgb(230, 75, 53)))
                g.FillPie(b1, 1.5f, 1.5f, 21f, 21f, 180, 110);
            using (var b2 = new SolidBrush(Color.FromArgb(77, 187, 213)))
                g.FillPie(b2, 1.5f, 1.5f, 21f, 21f, 290, 110);
            using (var b3 = new SolidBrush(Color.FromArgb(0, 160, 135)))
                g.FillPie(b3, 1.5f, 1.5f, 21f, 21f, 40, 140);

            using (var seamPen = new Pen(Color.FromArgb(25, 25, 30), 2.0f))
            {
                g.DrawLine(seamPen, 12, 12, 2, 12);
                g.DrawLine(seamPen, 12, 12, 16, 2.5f);
                g.DrawLine(seamPen, 12, 12, 20, 19);
                g.DrawEllipse(seamPen, 1.5f, 1.5f, 21f, 21f);
            }
            using (var normPen = new Pen(Color.White, 1.6f))
            {
                DrawArrow(g, normPen, 8, 9, 5, 4);
                DrawArrow(g, normPen, 15, 9, 19, 5);
                DrawArrow(g, normPen, 12, 16, 12, 21);
            }
        }

        // CurvSeg: Concave (Blue) vs Convex (Red) split by crisp white H=0 inflection S-curve
        private static void DrawCurvSeg(Graphics g)
        {
            using (var cvxBrush = new SolidBrush(Color.FromArgb(225, 65, 55)))
                g.FillPie(cvxBrush, 1.5f, 1.5f, 21f, 21f, 135, 180);
            using (var ccvBrush = new SolidBrush(Color.FromArgb(45, 135, 220)))
                g.FillPie(ccvBrush, 1.5f, 1.5f, 21f, 21f, -45, 180);

            using (var cutPen = new Pen(Color.White, 2.5f))
            {
                g.DrawBezier(cutPen, new PointF(4, 19), new PointF(10, 7), new PointF(14, 17), new PointF(20, 5));
            }
            using (var border = new Pen(Color.FromArgb(25, 25, 30), 1.2f))
                g.DrawEllipse(border, 1.5f, 1.5f, 21f, 21f);
        }

        // ScalarSeg: Segmented concentric scalar bands with dark seams
        private static void DrawScalarSeg(Graphics g)
        {
            using (var bOuter = new SolidBrush(Color.FromArgb(60, 130, 195)))
                g.FillEllipse(bOuter, 1.5f, 1.5f, 21f, 21f);
            using (var bMid = new SolidBrush(Color.FromArgb(0, 175, 145)))
                g.FillEllipse(bMid, 5.0f, 5.0f, 14f, 14f);
            using (var bInner = new SolidBrush(Color.FromArgb(245, 175, 55)))
                g.FillEllipse(bInner, 8.5f, 8.5f, 7f, 7f);

            using (var seamPen = new Pen(Color.FromArgb(25, 28, 35), 1.6f))
            {
                g.DrawEllipse(seamPen, 1.5f, 1.5f, 21f, 21f);
                g.DrawEllipse(seamPen, 5.0f, 5.0f, 14f, 14f);
                g.DrawEllipse(seamPen, 8.5f, 8.5f, 7f, 7f);
            }
        }

        // VecFamily: Quantized vector families on segmented zones
        private static void DrawVecFamily(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(30, 36, 50), Color.FromArgb(18, 22, 32));
            using (var p1 = new Pen(Color.FromArgb(255, 90, 75), 1.9f))
            using (var p2 = new Pen(Color.FromArgb(65, 215, 255), 1.9f))
            using (var p3 = new Pen(Color.FromArgb(120, 240, 110), 1.9f))
            {
                g.DrawLine(p1, 4, 10, 10, 5);
                g.DrawLine(p1, 5, 15, 11, 10);
                g.DrawLine(p2, 11, 16, 19, 16);
                g.DrawLine(p2, 11, 19.5f, 19, 19.5f);
                g.DrawLine(p3, 14, 11, 18, 4);
            }
        }

        // MoldParting: Multi-piece mold + gold parting line + pull vectors
        private static void DrawMoldParting(Graphics g)
        {
            using (var topMold = new SolidBrush(Color.FromArgb(70, 185, 135)))
                g.FillPie(topMold, 1.5f, 1.5f, 21f, 21f, 180, 180);
            using (var botMold = new SolidBrush(Color.FromArgb(65, 135, 210)))
                g.FillPie(botMold, 1.5f, 1.5f, 21f, 21f, 0, 180);
            using (var partingPen = new Pen(Color.Gold, 2.4f))
                g.DrawLine(partingPen, 1.5f, 12f, 22.5f, 12f);
            using (var pullPen = new Pen(Color.White, 1.6f))
            {
                DrawArrow(g, pullPen, 12, 9, 12, 3);
                DrawArrow(g, pullPen, 12, 15, 12, 21);
            }
            using (var border = new Pen(Color.FromArgb(25, 25, 30), 1.1f))
                g.DrawEllipse(border, 1.5f, 1.5f, 21f, 21f);
        }

        // MillingPaths: Concentric toolpaths + offset ball-mill frame
        private static void DrawMillingPaths(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(28, 38, 54), Color.FromArgb(16, 22, 34));
            using (var pathPen = new Pen(Color.FromArgb(0, 235, 185), 1.6f))
            {
                g.DrawEllipse(pathPen, 3.5f, 3.5f, 17f, 17f);
                g.DrawEllipse(pathPen, 6.8f, 6.8f, 10.4f, 10.4f);
                g.DrawEllipse(pathPen, 10.0f, 10.0f, 4.0f, 4.0f);
            }
            using (var toolPen = new Pen(Color.FromArgb(255, 190, 45), 1.9f))
                DrawArrow(g, toolPen, 15, 12, 20, 5);
        }

        // RollSphere: Tangent rolling spheres on surface
        private static void DrawRollSphere(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(25, 45, 70), Color.FromArgb(15, 25, 45));
            using (var surfPen = new Pen(Color.FromArgb(255, 195, 60), 2.1f))
                g.DrawBezier(surfPen, new PointF(2, 18), new PointF(8, 10), new PointF(16, 20), new PointF(22, 12));
            using (var sphBrush = new SolidBrush(Color.FromArgb(115, 0, 220, 255)))
            using (var sphPen = new Pen(Color.FromArgb(235, 120, 245, 255), 1.3f))
            {
                g.FillEllipse(sphBrush, 5, 4, 9, 9);
                g.DrawEllipse(sphPen, 5, 4, 9, 9);
                g.FillEllipse(sphBrush, 13, 6, 7, 7);
                g.DrawEllipse(sphPen, 13, 6, 7, 7);
            }
        }

        // IsoBoxSub: 3D Heat Offset shell + Subtractive Iso-Boxes
        private static void DrawIsoBoxSub(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(36, 42, 58), Color.FromArgb(20, 24, 34));
            using (var boxBrush = new SolidBrush(Color.FromArgb(185, 235, 115, 55)))
            using (var boxPen = new Pen(Color.FromArgb(255, 245, 205, 120), 1.1f))
            {
                g.FillRectangle(boxBrush, 4, 12, 5, 6);
                g.DrawRectangle(boxPen, 4, 12, 5, 6);
                g.FillRectangle(boxBrush, 9, 8, 5, 10);
                g.DrawRectangle(boxPen, 9, 8, 5, 10);
                g.FillRectangle(boxBrush, 14, 5, 5, 13);
                g.DrawRectangle(boxPen, 14, 5, 5, 13);
            }
            using (var isoPen = new Pen(Color.FromArgb(0, 235, 255), 1.8f))
            {
                g.DrawArc(isoPen, 2.5f, 2.5f, 19f, 19f, 180, 180);
            }
        }

        // Skeleton: 3D Medial Axis Skeleton branching graph + cyan medial nodes
        private static void DrawSkeleton(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(42, 24, 58), Color.FromArgb(22, 12, 34));
            using (var skelPen = new Pen(Color.FromArgb(255, 95, 220), 2.3f))
            {
                g.DrawLine(skelPen, 5, 18, 10, 12);
                g.DrawLine(skelPen, 10, 12, 15, 12);
                g.DrawLine(skelPen, 10, 12, 8, 5);
                g.DrawLine(skelPen, 15, 12, 19, 6);
                g.DrawLine(skelPen, 15, 12, 19, 18);
            }
            PointF[] nodes = { new PointF(5, 18), new PointF(10, 12), new PointF(15, 12), new PointF(8, 5), new PointF(19, 6), new PointF(19, 18) };
            foreach (var n in nodes)
                g.FillEllipse(Brushes.Cyan, n.X - 1.8f, n.Y - 1.8f, 3.6f, 3.6f);
        }

        // CleanCloud: Denoised cyan point cloud + normal ticks + rejected red outlier dots
        private static void DrawCleanCloud(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(22, 48, 58), Color.FromArgb(12, 26, 34));
            using (var surfPen = new Pen(Color.FromArgb(120, 0, 240, 195), 1.4f))
            {
                g.DrawEllipse(surfPen, 5.5f, 5.5f, 13f, 13f);
            }
            for (int k = 0; k < 8; k++)
            {
                double a = k * Math.PI / 4.0;
                float px = 12f + (float)(6.5 * Math.Cos(a));
                float py = 12f + (float)(6.5 * Math.Sin(a));
                g.FillEllipse(Brushes.Cyan, px - 1.6f, py - 1.6f, 3.2f, 3.2f);
            }
            // Outliers in red
            g.FillEllipse(Brushes.Crimson, 3f, 5f, 2.8f, 2.8f);
            g.FillEllipse(Brushes.Crimson, 18.5f, 17.5f, 2.8f, 2.8f);
        }

        // MCFlow: Jagged polygon -> Smooth cyan curve
        private static void DrawMCFlow(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(45, 55, 75), Color.FromArgb(25, 35, 50));
            PointF[] jagged = { new PointF(4, 15), new PointF(7, 6), new PointF(11, 13), new PointF(15, 5), new PointF(20, 14) };
            using (var p1 = new Pen(Color.FromArgb(150, 255, 100, 90), 1.2f))
                g.DrawLines(p1, jagged);
            using (var p2 = new Pen(Color.FromArgb(80, 230, 190), 2.3f))
                g.DrawArc(p2, 4, 8, 16, 12, 190, 160);
        }

        // OptField: Torus Optimal Direction Field + Singularities
        private static void DrawOptField(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(25, 32, 48), Color.FromArgb(45, 62, 88));
            using (var p1 = new Pen(Color.FromArgb(80, 225, 255), 1.4f))
            {
                g.DrawEllipse(p1, 4f, 4f, 16f, 16f);
                g.DrawEllipse(p1, 7.5f, 7.5f, 9f, 9f);
            }
            using (var p2 = new Pen(Color.FromArgb(165, 255, 120), 1.2f))
            {
                for (int i = 0; i < 8; i++)
                {
                    double a = i * Math.PI / 4.0;
                    float x1 = 12 + (float)(4.5 * Math.Cos(a)), y1 = 12 + (float)(4.5 * Math.Sin(a));
                    float x2 = 12 + (float)(9.5 * Math.Cos(a)), y2 = 12 + (float)(9.5 * Math.Sin(a));
                    g.DrawLine(p2, x1, y1, x2, y2);
                }
            }
            g.FillEllipse(Brushes.Red, 2.0f, 10f, 3.6f, 3.6f);
            g.FillEllipse(Brushes.Red, 18.4f, 10f, 3.6f, 3.6f);
        }

        // CrvStripes / CrvField: Guide Curve (Magenta) + Vector Segments & Stripes
        private static void DrawCrvStripes(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(245, 245, 248), Color.FromArgb(215, 218, 226));
            using (var guidePen = new Pen(Color.FromArgb(230, 35, 90), 2.5f))
            {
                g.DrawBezier(guidePen, new PointF(5, 3), new PointF(8, 9), new PointF(4, 15), new PointF(7, 21));
            }
            using (var stripePen = new Pen(Color.FromArgb(25, 30, 42), 1.8f))
            {
                g.DrawBezier(stripePen, new PointF(10.5f, 3), new PointF(13.5f, 9), new PointF(9.5f, 15), new PointF(12.5f, 21));
                g.DrawBezier(stripePen, new PointF(15.5f, 3), new PointF(18.5f, 9), new PointF(14.5f, 15), new PointF(17.5f, 21));
            }
        }

        // CrossQuads: Orthogonal U (Blue) and V (Red) Quad Stripes
        private static void DrawCrossQuads(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(240, 244, 250), Color.FromArgb(200, 212, 230));
            using (var pU = new Pen(Color.FromArgb(0, 115, 220), 1.5f))
            using (var pV = new Pen(Color.FromArgb(225, 55, 45), 1.5f))
            {
                for (int i = -1; i <= 1; i++)
                {
                    float off = i * 4.5f;
                    g.DrawBezier(pU, new PointF(3, 12 + off), new PointF(9, 9 + off), new PointF(15, 15 + off), new PointF(21, 12 + off));
                    g.DrawBezier(pV, new PointF(12 + off, 3), new PointF(9 + off, 9), new PointF(15 + off, 15), new PointF(12 + off, 21));
                }
            }
        }

        // MeshSeg / GeoSeam: Segmented patches with glowing cyan Geodesic Costura Seam
        private static void DrawMeshSeg(Graphics g)
        {
            using (var b1 = new SolidBrush(Color.FromArgb(230, 75, 53)))
                g.FillPie(b1, 1.5f, 1.5f, 21f, 21f, 180, 110);
            using (var b2 = new SolidBrush(Color.FromArgb(77, 187, 213)))
                g.FillPie(b2, 1.5f, 1.5f, 21f, 21f, 290, 110);
            using (var b3 = new SolidBrush(Color.FromArgb(0, 160, 135)))
                g.FillPie(b3, 1.5f, 1.5f, 21f, 21f, 40, 140);

            using (var seamPen = new Pen(Color.FromArgb(0, 255, 215), 2.3f))
            {
                g.DrawLine(seamPen, 12, 12, 2, 12);
                g.DrawLine(seamPen, 12, 12, 16, 2.5f);
                g.DrawLine(seamPen, 12, 12, 20, 19);
            }
            using (var border = new Pen(Color.FromArgb(25, 25, 30), 1.2f))
                g.DrawEllipse(border, 1.5f, 1.5f, 21f, 21f);
            g.FillEllipse(Brushes.Gold, 10.2f, 10.2f, 3.6f, 3.6f);
        }

        private static void DrawArrow(Graphics g, Pen pen, float x1, float y1, float x2, float y2)
        {
            g.DrawLine(pen, x1, y1, x2, y2);
            double ang = Math.Atan2(y2 - y1, x2 - x1);
            float headLen = 3.2f;
            float ax = x2 - (float)(headLen * Math.Cos(ang - 0.5));
            float ay = y2 - (float)(headLen * Math.Sin(ang - 0.5));
            float bx = x2 - (float)(headLen * Math.Cos(ang + 0.5));
            float by = y2 - (float)(headLen * Math.Sin(ang + 0.5));
            g.DrawLine(pen, x2, y2, ax, ay);
            g.DrawLine(pen, x2, y2, bx, by);
        }
    }
}
