using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace CraneGH.Core
{
    /// <summary>
    /// Procedural 24x24 anti-aliased icon renderer designed in Keenan Crane's signature visual style
    /// (Heat Method crimson-gold isolines, Trivial Connections red singularity + blue ticks,
    /// Stripe Patterns Y-bifurcation zebra stripes, FlipOut blue-gold mesh, Polyscope segmentation palettes).
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
                    case "Curvatures": DrawCurvatures(g); break;
                    case "MCFlow": DrawMCFlow(g); break;
                    case "GradLap": DrawGradLap(g); break;
                    case "Harmonic": DrawHarmonic(g); break;
                    case "HeatDist": DrawHeatDist(g); break;
                    case "IsoCrvs": DrawIsoCrvs(g); break;
                    case "GeoPath": DrawGeoPath(g); break;
                    case "VecHeat": DrawVecHeat(g); break;
                    case "LogMap": DrawLogMap(g); break;
                    case "DecalMap": DrawDecalMap(g); break;
                    case "OptField": DrawOptField(g); break;
                    case "TrivConn": DrawTrivConn(g); break;
                    case "Streamlines": DrawStreamlines(g); break;
                    case "Stripes": DrawStripes(g); break;
                    case "CrvStripes": DrawCrvStripes(g); break;
                    case "SpinXForm": DrawSpinXForm(g); break;
                    case "BFF": DrawBFF(g); break;
                    case "Auxetics": DrawAuxetics(g); break;
                    case "FlipOut": DrawFlipOut(g); break;
                    case "RepCurve": DrawRepCurve(g); break;
                    case "WoS": DrawWoS(g); break;
                    case "SubgridTet": DrawSubgridTet(g); break;
                    case "SharpEdges": DrawSharpEdges(g); break;
                    case "FeatHeat": DrawFeatHeat(g); break;
                    case "MeshSeg": DrawMeshSeg(g); break;
                    case "CrossField": DrawCrossField(g); break;
                    case "CrossQuads": DrawCrossQuads(g); break;
                    case "VarCuts": DrawVarCuts(g); break;
                    case "SmoothScalar": DrawSmoothScalar(g); break;
                    case "SmoothField": DrawSmoothField(g); break;
                    case "VecView": DrawVecView(g); break;
                    case "ScalarView": DrawScalarView(g); break;
                    case "WalkOnMesh": DrawWalkOnMesh(g); break;
                    case "IsoCut": DrawIsoCut(g); break;
                    case "VecFamily": DrawVecFamily(g); break;
                    case "RollSphere": DrawRollSphere(g); break;
                    case "KMeansSeg": DrawKMeansSeg(g); break;
                    case "MoldParting": DrawMoldParting(g); break;
                    case "MillingPaths": DrawMillingPaths(g); break;
                    case "IsoBoxSub": DrawIsoBoxSub(g); break;
                    case "GeoTSP": DrawGeoTSP(g); break;
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
            using (var pen = new Pen(Color.FromArgb(180, 30, 30, 35), 1.0f))
            {
                g.DrawEllipse(pen, 1.5f, 1.5f, 21f, 21f);
            }
        }

        // 01. Discrete Curvatures
        private static void DrawCurvatures(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 110, 200), Color.FromArgb(220, 60, 50));
            using (var pen = new Pen(Color.FromArgb(230, 255, 255, 255), 1.3f))
            {
                g.DrawArc(pen, 4f, 8f, 16f, 8f, 0, 180);
                g.DrawArc(pen, 8f, 4f, 8f, 16f, 90, 180);
            }
            using (var p2 = new Pen(Color.Gold, 1.5f))
            {
                g.DrawLine(p2, 12, 3, 12, 21);
                g.DrawLine(p2, 3, 12, 21, 12);
            }
        }

        // 02. Mean Curvature Smooth
        private static void DrawMCFlow(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(45, 55, 75), Color.FromArgb(25, 35, 50));
            PointF[] jagged = { new PointF(4, 15), new PointF(7, 6), new PointF(11, 13), new PointF(15, 5), new PointF(20, 14) };
            using (var p1 = new Pen(Color.FromArgb(150, 255, 100, 90), 1.2f))
                g.DrawLines(p1, jagged);
            using (var p2 = new Pen(Color.FromArgb(80, 230, 190), 2.2f))
                g.DrawArc(p2, 4, 8, 16, 12, 190, 160);
        }

        // 03. Scalar Field Calculus
        private static void DrawGradLap(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(255, 210, 60), Color.FromArgb(190, 40, 40));
            using (var pen = new Pen(Color.White, 1.6f))
            {
                DrawArrow(g, pen, 6, 17, 17, 6);
                DrawArrow(g, pen, 6, 11, 13, 4);
                DrawArrow(g, pen, 11, 18, 18, 11);
            }
        }

        // 04. Harmonic Scalar Field
        private static void DrawHarmonic(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 180, 220), Color.FromArgb(250, 130, 50));
            using (var pen = new Pen(Color.FromArgb(180, 255, 255, 255), 1.1f))
            {
                g.DrawBezier(pen, new PointF(4, 12), new PointF(9, 6), new PointF(15, 18), new PointF(20, 12));
                g.DrawBezier(pen, new PointF(5, 16), new PointF(10, 10), new PointF(14, 20), new PointF(19, 15));
            }
            g.FillEllipse(Brushes.White, 4, 5, 4, 4);
            g.FillEllipse(Brushes.Black, 16, 15, 4, 4);
        }

        // 05. Heat Geodesic Distance (Keenan Crane's Crimson-Orange-Yellow-White Bunny Heatmap)
        private static void DrawHeatDist(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(120, 15, 20), Color.FromArgb(210, 45, 30));
            g.FillEllipse(new SolidBrush(Color.FromArgb(245, 120, 30)), 3f, 3f, 15f, 15f);
            g.FillEllipse(new SolidBrush(Color.FromArgb(255, 200, 60)), 5f, 5f, 10.5f, 10.5f);
            g.FillEllipse(Brushes.White, 7.5f, 7.5f, 5f, 5f);

            using (var pen = new Pen(Color.FromArgb(210, 255, 240, 200), 1.0f))
            {
                g.DrawEllipse(pen, 5f, 5f, 10.5f, 10.5f);
                g.DrawEllipse(pen, 3f, 3f, 15f, 15f);
                g.DrawEllipse(pen, 1.8f, 1.8f, 19.5f, 19.5f);
            }
            g.FillEllipse(Brushes.Red, 9f, 9f, 2.5f, 2.5f);
        }

        // 06. Mesh Iso-Contours
        private static void DrawIsoCrvs(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(50, 30, 40), Color.FromArgb(130, 30, 40));
            using (var pen = new Pen(Color.FromArgb(255, 225, 110), 1.5f))
            {
                g.DrawEllipse(pen, 3f, 3f, 18f, 18f);
                g.DrawEllipse(pen, 6f, 6f, 12f, 12f);
                g.DrawEllipse(pen, 9f, 9f, 6f, 6f);
            }
        }

        // 07. Geodesic Shortest Path
        private static void DrawGeoPath(Graphics g)
        {
            DrawHeatDist(g);
            using (var pen = new Pen(Color.FromArgb(0, 255, 220), 2.4f))
            {
                g.DrawBezier(pen, new PointF(10, 10), new PointF(14, 8), new PointF(15, 16), new PointF(19, 18));
            }
            g.FillEllipse(Brushes.Cyan, 17.5f, 16.5f, 3.5f, 3.5f);
        }

        // 08. Vector Parallel Transport
        private static void DrawVecHeat(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(35, 95, 165), Color.FromArgb(20, 50, 95));
            using (var pen = new Pen(Color.FromArgb(160, 225, 255), 1.5f))
            {
                DrawArrow(g, pen, 5, 16, 10, 8);
                DrawArrow(g, pen, 10, 18, 15, 10);
                DrawArrow(g, pen, 15, 16, 20, 8);
            }
            using (var srcPen = new Pen(Color.FromArgb(255, 190, 50), 2.0f))
            {
                DrawArrow(g, srcPen, 4, 12, 9, 4);
            }
        }

        // 09. Logarithmic Map UV (Rainbow Polar Grid)
        private static void DrawLogMap(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(235, 70, 150), Color.FromArgb(60, 200, 230));
            using (var pen = new Pen(Color.FromArgb(220, 255, 255, 255), 1.1f))
            {
                g.DrawEllipse(pen, 5f, 5f, 14f, 14f);
                g.DrawEllipse(pen, 8.5f, 8.5f, 7f, 7f);
                g.DrawLine(pen, 12, 2, 12, 22);
                g.DrawLine(pen, 2, 12, 22, 12);
                g.DrawLine(pen, 5, 5, 19, 19);
                g.DrawLine(pen, 19, 5, 5, 19);
            }
            g.FillEllipse(Brushes.Black, 10.5f, 10.5f, 3f, 3f);
        }

        // 10. Surface Decal Mapper
        private static void DrawDecalMap(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(50, 140, 210), Color.FromArgb(30, 70, 130));
            using (var pen = new Pen(Color.FromArgb(255, 230, 80), 1.6f))
            {
                g.DrawRectangle(pen, 6, 6, 12, 12);
                g.DrawLine(pen, 6, 12, 18, 12);
                g.DrawLine(pen, 12, 6, 12, 18);
            }
        }

        // 11. Optimal Direction Field (Torus Cross-Field + Red/Blue Singularities)
        private static void DrawOptField(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(25, 30, 40), Color.FromArgb(45, 60, 85));
            using (var p1 = new Pen(Color.FromArgb(80, 220, 255), 1.3f))
            {
                g.DrawEllipse(p1, 4f, 4f, 16f, 16f);
                g.DrawEllipse(p1, 7.5f, 7.5f, 9f, 9f);
            }
            using (var p2 = new Pen(Color.FromArgb(160, 255, 120), 1.1f))
            {
                for (int i = 0; i < 8; i++)
                {
                    double a = i * Math.PI / 4.0;
                    float x1 = 12 + (float)(4.5 * Math.Cos(a)), y1 = 12 + (float)(4.5 * Math.Sin(a));
                    float x2 = 12 + (float)(9.5 * Math.Cos(a)), y2 = 12 + (float)(9.5 * Math.Sin(a));
                    g.DrawLine(p2, x1, y1, x2, y2);
                }
            }
            g.FillEllipse(Brushes.Red, 1.5f, 10f, 4f, 4f);
            g.FillEllipse(Brushes.Red, 18.5f, 10f, 4f, 4f);
        }

        // 12. Trivial Connections (Directly inspired by TrivialConnections/test.png: Grey sphere, Red center dot, Blue ticks)
        private static void DrawTrivConn(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(230, 232, 235), Color.FromArgb(175, 180, 190));
            using (var bluePen = new Pen(Color.FromArgb(0, 70, 255), 1.6f))
            {
                for (int ring = 1; ring <= 2; ring++)
                {
                    float r = ring * 4.2f;
                    int count = ring * 6;
                    for (int k = 0; k < count; k++)
                    {
                        double a = 2.0 * Math.PI * k / count;
                        float cx = 12f + (float)(r * Math.Cos(a));
                        float cy = 12f + (float)(r * Math.Sin(a));
                        float tx = (float)(-Math.Sin(a)) * 1.8f;
                        float ty = (float)(Math.Cos(a)) * 1.8f;
                        g.DrawLine(bluePen, cx - tx, cy - ty, cx + tx, cy + ty);
                    }
                }
            }
            g.FillEllipse(new SolidBrush(Color.FromArgb(235, 50, 40)), 9.5f, 9.5f, 5f, 5f);
        }

        // 13. Mesh Field Streamlines
        private static void DrawStreamlines(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(30, 65, 110), Color.FromArgb(15, 35, 65));
            using (var pen = new Pen(Color.FromArgb(255, 185, 65), 1.6f))
            {
                g.DrawBezier(pen, new PointF(3, 8), new PointF(9, 3), new PointF(15, 13), new PointF(21, 8));
                g.DrawBezier(pen, new PointF(3, 13), new PointF(9, 8), new PointF(15, 18), new PointF(21, 13));
                g.DrawBezier(pen, new PointF(3, 18), new PointF(9, 13), new PointF(15, 23), new PointF(21, 18));
            }
        }

        // 14. Stripe Patterns (Zebra Stripes with Topological Y-Bifurcation & Red Branch Dot)
        private static void DrawStripes(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(245, 245, 245), Color.FromArgb(215, 215, 220));
            using (var blackPen = new Pen(Color.FromArgb(25, 25, 30), 2.3f))
            {
                g.DrawLine(blackPen, 4, 5, 20, 5);
                // Y-Bifurcation stripe in the center!
                g.DrawLine(blackPen, 3, 12, 11, 12);
                g.DrawLine(blackPen, 11, 12, 19, 8.5f);
                g.DrawLine(blackPen, 11, 12, 19, 15.5f);
                g.DrawLine(blackPen, 4, 19, 20, 19);
            }
            g.FillEllipse(new SolidBrush(Color.FromArgb(235, 50, 40)), 9.5f, 10.5f, 4f, 4f);
        }

        // 15. Curve Guided Equidistant Stripes (Guide curve + equidistant parallel stripes at spacing D)
        private static void DrawCrvStripes(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(245, 245, 245), Color.FromArgb(215, 215, 220));
            // Guide curve in bright Crimson/Magenta on the left
            using (var guidePen = new Pen(Color.FromArgb(230, 40, 90), 2.4f))
            {
                g.DrawBezier(guidePen, new PointF(5, 3), new PointF(8, 9), new PointF(4, 15), new PointF(7, 21));
            }
            // Equidistant stripes at fixed spacing D
            using (var stripePen = new Pen(Color.FromArgb(25, 30, 40), 1.8f))
            {
                g.DrawBezier(stripePen, new PointF(10, 3), new PointF(13, 9), new PointF(9, 15), new PointF(12, 21));
                g.DrawBezier(stripePen, new PointF(15, 3), new PointF(18, 9), new PointF(14, 15), new PointF(17, 21));
                g.DrawBezier(stripePen, new PointF(19.5f, 4), new PointF(21.5f, 9), new PointF(18.5f, 15), new PointF(20.5f, 20));
            }
        }

        // 16. Spin Conformal Deform (Green & Purple SpinXForm)
        private static void DrawSpinXForm(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(80, 190, 110), Color.FromArgb(165, 65, 175));
            using (var pen = new Pen(Color.FromArgb(210, 255, 255, 255), 1.1f))
            {
                g.DrawEllipse(pen, 4, 6, 16, 12);
                g.DrawEllipse(pen, 6, 4, 12, 16);
            }
        }

        // 17. Conformal Flatten BFF (Checkerboard UV Grid)
        private static void DrawBFF(Graphics g)
        {
            using (var bBg = new SolidBrush(Color.FromArgb(240, 240, 245)))
                g.FillRectangle(bBg, 2, 2, 20, 20);
            using (var dark = new SolidBrush(Color.FromArgb(35, 40, 50)))
            {
                for (int ix = 0; ix < 4; ix++)
                    for (int iy = 0; iy < 4; iy++)
                        if ((ix + iy) % 2 == 0)
                            g.FillRectangle(dark, 2 + ix * 5, 2 + iy * 5, 5, 5);
            }
            using (var border = new Pen(Color.FromArgb(0, 170, 230), 1.5f))
                g.DrawRectangle(border, 2, 2, 20, 20);
        }

        // 18. Deployable Auxetics (Rotating Squares Kirigami)
        private static void DrawAuxetics(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(235, 240, 248), Color.FromArgb(190, 205, 225));
            using (var br = new SolidBrush(Color.FromArgb(55, 115, 185)))
            using (var pen = new Pen(Color.FromArgb(20, 45, 80), 1.0f))
            {
                PointF[][] quads = new PointF[][]
                {
                    new[] { new PointF(6, 4), new PointF(12, 6), new PointF(10, 12), new PointF(4, 10) },
                    new[] { new PointF(12, 6), new PointF(18, 4), new PointF(20, 10), new PointF(14, 12) },
                    new[] { new PointF(4, 14), new PointF(10, 12), new PointF(12, 18), new PointF(6, 20) },
                    new[] { new PointF(14, 12), new PointF(20, 14), new PointF(18, 20), new PointF(12, 18) }
                };
                foreach (var q in quads)
                {
                    g.FillPolygon(br, q);
                    g.DrawPolygon(pen, q);
                }
            }
        }

        // 19. FlipOut Geodesic Loop Shorten (Keenan's Blue Mesh + Gold Edges + Red Geodesic)
        private static void DrawFlipOut(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(0, 140, 235), Color.FromArgb(0, 95, 180));
            using (var goldPen = new Pen(Color.FromArgb(255, 195, 40), 1.1f))
            {
                g.DrawLine(goldPen, 3, 12, 12, 3);
                g.DrawLine(goldPen, 12, 3, 21, 12);
                g.DrawLine(goldPen, 21, 12, 12, 21);
                g.DrawLine(goldPen, 12, 21, 3, 12);
                g.DrawLine(goldPen, 3, 12, 21, 12);
                g.DrawLine(goldPen, 12, 3, 12, 21);
            }
            using (var redPen = new Pen(Color.FromArgb(245, 30, 35), 2.4f))
            {
                g.DrawLine(redPen, 5, 18, 19, 6);
            }
            g.FillEllipse(Brushes.DarkSlateGray, 3.5f, 16.5f, 3.5f, 3.5f);
            g.FillEllipse(Brushes.DarkSlateGray, 17f, 4.5f, 3.5f, 3.5f);
        }

        // 20. Repulsive Curve Flow (Orange & Purple Interlocking Repulsive Tubes)
        private static void DrawRepCurve(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 30, 50), Color.FromArgb(25, 20, 35));
            using (var p1 = new Pen(Color.FromArgb(255, 150, 40), 2.5f))
                g.DrawArc(p1, 4, 4, 16, 10, 0, 270);
            using (var p2 = new Pen(Color.FromArgb(165, 85, 235), 2.5f))
                g.DrawArc(p2, 4, 10, 16, 10, 180, 270);
        }

        // 21. Walk on Spheres PDE
        private static void DrawWoS(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(30, 45, 65), Color.FromArgb(15, 25, 40));
            using (var dashPen = new Pen(Color.FromArgb(120, 225, 255), 1.2f) { DashStyle = DashStyle.Dot })
            {
                g.DrawEllipse(dashPen, 3, 5, 11, 11);
                g.DrawEllipse(dashPen, 10, 7, 9, 9);
            }
            using (var linePen = new Pen(Color.FromArgb(255, 200, 70), 1.5f))
            {
                g.DrawLine(linePen, 8.5f, 10.5f, 14.5f, 11.5f);
                g.DrawLine(linePen, 14.5f, 11.5f, 19f, 11.5f);
            }
            g.FillEllipse(Brushes.White, 7f, 9f, 3f, 3f);
            g.FillEllipse(Brushes.Red, 17.5f, 10f, 3f, 3f);
        }

        // 22. Subgrid Marching Tet
        private static void DrawSubgridTet(Graphics g)
        {
            PointF p0 = new PointF(12, 2), p1 = new PointF(2, 20), p2 = new PointF(22, 20), p3 = new PointF(12, 13);
            PointF[] sheet = { new PointF(7, 11), new PointF(17, 11), new PointF(15, 17), new PointF(9, 17) };
            using (var sb = new SolidBrush(Color.FromArgb(190, 0, 190, 230)))
                g.FillPolygon(sb, sheet);
            using (var tetPen = new Pen(Color.FromArgb(245, 180, 40), 1.4f))
            {
                g.DrawPolygon(tetPen, new[] { p0, p1, p2 });
                g.DrawLine(tetPen, p0, p3);
                g.DrawLine(tetPen, p1, p3);
                g.DrawLine(tetPen, p2, p3);
            }
        }

        // 23. Sharp Feature Creases (Aristas Fuertes)
        private static void DrawSharpEdges(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(65, 75, 95), Color.FromArgb(40, 45, 60));
            using (var weakPen = new Pen(Color.FromArgb(110, 170, 190, 215), 1.0f))
            {
                g.DrawLine(weakPen, 4, 12, 12, 4);
                g.DrawLine(weakPen, 12, 20, 20, 12);
            }
            using (var creasePen = new Pen(Color.FromArgb(255, 220, 40), 2.5f))
            {
                g.DrawLine(creasePen, 4, 18, 12, 6);
                g.DrawLine(creasePen, 12, 6, 20, 18);
            }
        }

        // 24. Feature-Aware Heatmap (Heatmap blocked by a strong diagonal crease barrier)
        private static void DrawFeatHeat(Graphics g)
        {
            // Upper-left warm heatmap, lower-right cool region separated by a sharp diagonal crease
            using (var warmBrush = new LinearGradientBrush(new PointF(2, 2), new PointF(14, 14), Color.FromArgb(255, 240, 80), Color.FromArgb(220, 45, 30)))
                g.FillPie(warmBrush, 1.5f, 1.5f, 21f, 21f, 135, 180);
            using (var coolBrush = new SolidBrush(Color.FromArgb(40, 75, 145)))
                g.FillPie(coolBrush, 1.5f, 1.5f, 21f, 21f, -45, 180);

            using (var barrierPen = new Pen(Color.White, 2.2f))
                g.DrawLine(barrierPen, 4.5f, 19.5f, 19.5f, 4.5f);
            using (var border = new Pen(Color.FromArgb(30, 30, 35), 1.1f))
                g.DrawEllipse(border, 1.5f, 1.5f, 21f, 21f);
        }

        // 25. Mesh Heatmap Segmenter (Multi-colored segmented shards with crisp dark crease seams)
        private static void DrawMeshSeg(Graphics g)
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
        }

        // 26. Principal Cross Field (4-RoSy Crosses on Surface)
        private static void DrawCrossField(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(30, 50, 85), Color.FromArgb(18, 30, 55));
            using (var pU = new Pen(Color.FromArgb(80, 225, 255), 1.5f))
            using (var pV = new Pen(Color.FromArgb(255, 120, 90), 1.5f))
            {
                PointF[] centers = { new PointF(7, 7), new PointF(17, 7), new PointF(12, 12), new PointF(7, 17), new PointF(17, 17) };
                foreach (var c in centers)
                {
                    g.DrawLine(pU, c.X - 2.6f, c.Y - 0.8f, c.X + 2.6f, c.Y + 0.8f);
                    g.DrawLine(pV, c.X - 0.8f, c.Y + 2.6f, c.X + 0.8f, c.Y - 2.6f);
                }
            }
            g.FillEllipse(Brushes.Gold, 10.5f, 10.5f, 3f, 3f);
        }

        // 27. Cross Field Quad Grid (Orthogonal Equidistant U/V Curves)
        private static void DrawCrossQuads(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(240, 244, 250), Color.FromArgb(200, 212, 230));
            using (var pU = new Pen(Color.FromArgb(0, 110, 215), 1.4f))
            using (var pV = new Pen(Color.FromArgb(225, 55, 45), 1.4f))
            {
                for (int i = -1; i <= 1; i++)
                {
                    float off = i * 4.5f;
                    g.DrawBezier(pU, new PointF(3, 12 + off), new PointF(9, 9 + off), new PointF(15, 15 + off), new PointF(21, 12 + off));
                    g.DrawBezier(pV, new PointF(12 + off, 3), new PointF(9 + off, 9), new PointF(15 + off, 15), new PointF(12 + off, 21));
                }
            }
        }

        // 28. Variational Surface Cuts (Sharp & Crane 2018: Weighted Seam Cut through Gaussian Curvature Cones)
        private static void DrawVarCuts(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(235, 235, 240), Color.FromArgb(185, 195, 210));
            // Two Gaussian curvature / weight cones
            g.FillEllipse(new SolidBrush(Color.FromArgb(140, 255, 160, 50)), 4f, 5f, 7f, 7f);
            g.FillEllipse(new SolidBrush(Color.FromArgb(140, 255, 160, 50)), 13f, 12f, 7f, 7f);
            // Variational cut seam snapping through the weighted cones
            using (var cutPen = new Pen(Color.FromArgb(220, 25, 40), 2.4f))
            {
                g.DrawBezier(cutPen, new PointF(3, 16), new PointF(6, 5), new PointF(18, 19), new PointF(21, 8));
            }
            g.FillEllipse(Brushes.Red, 6f, 7f, 3f, 3f);
            g.FillEllipse(Brushes.Red, 15f, 14f, 3f, 3f);
        }

        // 29. Scalar Field Smooth (Noisy jagged wave -> Smooth glowing Gaussian wave)
        private static void DrawSmoothScalar(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(45, 55, 85), Color.FromArgb(25, 32, 52));
            // Faint noisy input signal
            using (var noisyPen = new Pen(Color.FromArgb(130, 255, 110, 90), 1.1f))
            {
                PointF[] jagged = {
                    new PointF(3, 15), new PointF(5, 8), new PointF(7, 17),
                    new PointF(9.5f, 5), new PointF(12, 13), new PointF(14.5f, 4.5f),
                    new PointF(17, 16), new PointF(19, 9), new PointF(21, 14)
                };
                g.DrawLines(noisyPen, jagged);
            }
            // Crisp smoothed output wave
            using (var smoothPen = new Pen(Color.FromArgb(90, 245, 210), 2.3f))
            {
                g.DrawBezier(smoothPen, new PointF(3, 15), new PointF(9, 4), new PointF(15, 4), new PointF(21, 15));
            }
        }

        // 30. Vector & N-RoSy Field Smooth (Aligned flowing tangent vectors via Connection Laplacian)
        private static void DrawSmoothField(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(28, 62, 95), Color.FromArgb(16, 36, 58));
            using (var flowPen = new Pen(Color.FromArgb(110, 255, 220, 100), 1.1f))
            {
                g.DrawBezier(flowPen, new PointF(3, 16), new PointF(9, 10), new PointF(15, 10), new PointF(21, 7));
                g.DrawBezier(flowPen, new PointF(3, 19), new PointF(9, 13), new PointF(15, 13), new PointF(21, 10));
            }
            using (var arrowPen = new Pen(Color.FromArgb(100, 245, 255), 1.6f))
            {
                DrawArrow(g, arrowPen, 4.5f, 15.5f, 9.5f, 11.5f);
                DrawArrow(g, arrowPen, 9.5f, 14.5f, 15.0f, 11.0f);
                DrawArrow(g, arrowPen, 14.5f, 12.5f, 19.8f, 8.5f);
            }
        }

        // 31. Mesh Vector & Field Display (Anchored tangent arrows & crosses on mesh vertices/faces)
        private static void DrawVecView(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(35, 42, 60), Color.FromArgb(20, 24, 36));
            using (var gridPen = new Pen(Color.FromArgb(80, 140, 165, 200), 0.9f))
            {
                g.DrawLine(gridPen, 4, 17, 12, 6);
                g.DrawLine(gridPen, 12, 6, 20, 17);
                g.DrawLine(gridPen, 4, 17, 20, 17);
            }
            using (var arrowPen = new Pen(Color.FromArgb(255, 210, 55), 1.7f))
            {
                DrawArrow(g, arrowPen, 6, 15, 11, 10);
                DrawArrow(g, arrowPen, 12, 15, 18, 10);
                DrawArrow(g, arrowPen, 10, 9, 16, 5);
            }
            g.FillEllipse(Brushes.Cyan, 4.5f, 13.5f, 3f, 3f);
            g.FillEllipse(Brushes.Cyan, 10.5f, 13.5f, 3f, 3f);
            g.FillEllipse(Brushes.Cyan, 8.5f, 7.5f, 3f, 3f);
        }

        // 32. Scalar Field Inspector & Legend Display
        private static void DrawScalarView(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(20, 40, 110), Color.FromArgb(240, 80, 35));
            using (var isoPen = new Pen(Color.FromArgb(220, 255, 245, 180), 1.2f))
            {
                g.DrawArc(isoPen, 4, 4, 16, 16, 200, 140);
                g.DrawArc(isoPen, 6, 7, 12, 12, 200, 140);
            }
            using (var barBrush = new LinearGradientBrush(new RectangleF(18, 4, 3.5f, 16), Color.Yellow, Color.Crimson, 90f))
            {
                g.FillRectangle(barBrush, 18f, 4f, 3.5f, 16f);
                g.DrawRectangle(Pens.White, 18, 4, 3, 16);
            }
        }

        // 33. Walk on Mesh & Scalar Gradient
        private static void DrawWalkOnMesh(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 75, 115), Color.FromArgb(20, 38, 65));
            PointF[] walk = { new PointF(4, 18), new PointF(8, 12), new PointF(13, 14), new PointF(16, 7), new PointF(20, 5) };
            using (var walkPen = new Pen(Color.FromArgb(255, 215, 50), 2.0f))
            {
                g.DrawLines(walkPen, walk);
            }
            foreach (var p in walk)
                g.FillEllipse(Brushes.White, p.X - 1.8f, p.Y - 1.8f, 3.6f, 3.6f);
            g.FillEllipse(Brushes.LimeGreen, walk[0].X - 2.2f, walk[0].Y - 2.2f, 4.4f, 4.4f);
            g.FillEllipse(Brushes.Crimson, walk[4].X - 2.2f, walk[4].Y - 2.2f, 4.4f, 4.4f);
        }

        // 34. Exact Iso-Cut & Band Slicer
        private static void DrawIsoCut(Graphics g)
        {
            using (var b1 = new SolidBrush(Color.FromArgb(235, 95, 55)))
                g.FillPie(b1, 1.5f, 1.5f, 21f, 21f, 160, 180);
            using (var b2 = new SolidBrush(Color.FromArgb(55, 165, 215)))
                g.FillPie(b2, 1.5f, 1.5f, 21f, 21f, -20, 180);
            using (var cutPen = new Pen(Color.Gold, 2.5f))
                g.DrawLine(cutPen, 2.5f, 14.5f, 21.5f, 9.5f);
            using (var border = new Pen(Color.FromArgb(30, 30, 35), 1.1f))
                g.DrawEllipse(border, 1.5f, 1.5f, 21f, 21f);
        }

        // 35. Vector Family Discretizer
        private static void DrawVecFamily(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(30, 36, 50), Color.FromArgb(18, 22, 32));
            using (var p1 = new Pen(Color.FromArgb(255, 90, 75), 1.8f))
            using (var p2 = new Pen(Color.FromArgb(65, 215, 255), 1.8f))
            using (var p3 = new Pen(Color.FromArgb(120, 240, 110), 1.8f))
            {
                DrawArrow(g, p1, 4, 10, 10, 5);
                DrawArrow(g, p1, 5, 15, 11, 10);
                DrawArrow(g, p2, 11, 16, 19, 16);
                DrawArrow(g, p2, 11, 20, 19, 20);
                DrawArrow(g, p3, 14, 11, 18, 4);
            }
        }

        // 36. Rolling Spheres & Collision-Free Offset
        private static void DrawRollSphere(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(25, 45, 70), Color.FromArgb(15, 25, 45));
            using (var surfPen = new Pen(Color.FromArgb(255, 195, 60), 2.0f))
                g.DrawBezier(surfPen, new PointF(2, 18), new PointF(8, 10), new PointF(16, 20), new PointF(22, 12));
            using (var sphBrush = new SolidBrush(Color.FromArgb(110, 0, 220, 255)))
            using (var sphPen = new Pen(Color.FromArgb(230, 120, 245, 255), 1.3f))
            {
                g.FillEllipse(sphBrush, 5, 4, 9, 9);
                g.DrawEllipse(sphPen, 5, 4, 9, 9);
                g.FillEllipse(sphBrush, 13, 6, 7, 7);
                g.DrawEllipse(sphPen, 13, 6, 7, 7);
            }
        }

        // 37. K-Means Normal & Vector Field Segmenter
        private static void DrawKMeansSeg(Graphics g)
        {
            DrawMeshSeg(g);
            using (var normPen = new Pen(Color.White, 1.7f))
            {
                DrawArrow(g, normPen, 8, 9, 5, 4);
                DrawArrow(g, normPen, 15, 9, 19, 5);
                DrawArrow(g, normPen, 12, 16, 12, 21);
            }
        }

        // 38. Mold Parting Line & Undercuts
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
        }

        // 39. Concentric & Spiral Milling Paths
        private static void DrawMillingPaths(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(30, 40, 55), Color.FromArgb(18, 24, 35));
            using (var pathPen = new Pen(Color.FromArgb(0, 230, 180), 1.5f))
            {
                g.DrawEllipse(pathPen, 3.5f, 3.5f, 17f, 17f);
                g.DrawEllipse(pathPen, 6.8f, 6.8f, 10.4f, 10.4f);
                g.DrawEllipse(pathPen, 10.0f, 10.0f, 4.0f, 4.0f);
            }
            using (var toolPen = new Pen(Color.FromArgb(255, 180, 40), 1.8f))
                DrawArrow(g, toolPen, 15, 12, 20, 5);
        }

        // 40. Iso-Box & Volumetric Shell Subtraction
        private static void DrawIsoBoxSub(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(40, 45, 60), Color.FromArgb(22, 26, 36));
            using (var boxBrush = new SolidBrush(Color.FromArgb(180, 235, 110, 60)))
            using (var boxPen = new Pen(Color.FromArgb(255, 240, 200, 120), 1.1f))
            {
                g.FillRectangle(boxBrush, 4, 12, 5, 6);
                g.DrawRectangle(boxPen, 4, 12, 5, 6);
                g.FillRectangle(boxBrush, 9, 8, 5, 10);
                g.DrawRectangle(boxPen, 9, 8, 5, 10);
                g.FillRectangle(boxBrush, 14, 5, 5, 13);
                g.DrawRectangle(boxPen, 14, 5, 5, 13);
            }
        }

        // 41. Geodesic TSP & 6-Axis Robot Path
        private static void DrawGeoTSP(Graphics g)
        {
            DrawDiscBackground(g, Color.FromArgb(32, 28, 52), Color.FromArgb(18, 15, 32));
            PointF[] tour = {
                new PointF(5, 7), new PointF(12, 4), new PointF(19, 8),
                new PointF(18, 16), new PointF(11, 19), new PointF(5, 15), new PointF(11, 11), new PointF(5, 7)
            };
            using (var tourPen = new Pen(Color.FromArgb(0, 240, 200), 1.8f))
                g.DrawLines(tourPen, tour);
            for (int i = 0; i < tour.Length - 1; i++)
                g.FillEllipse(Brushes.Gold, tour[i].X - 1.8f, tour[i].Y - 1.8f, 3.6f, 3.6f);
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
