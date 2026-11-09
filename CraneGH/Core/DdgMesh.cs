using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace CraneGH.Core
{
    public class DdgHalfEdge
    {
        public int Index;
        public int OriginVertex;
        public int TargetVertex;
        public int FaceIndex;
        public int EdgeIndex;
        public int Next;
        public int Prev;
        public int Twin = -1;
        public double Cotan;
        public double AngularCoordinate;
        public double CornerAngle;
    }

    public class DdgEdge
    {
        public int Index;
        public int V0;
        public int V1;
        public int HalfEdge0;
        public int HalfEdge1 = -1;
        public double Length;
        public double CotanWeight;
        public double DihedralAngle;       // [0, pi] angle between adjacent face normals
        public double SignedDihedralAngle; // positive = convex ridge, negative = concave valley
        public bool IsBoundary => HalfEdge1 == -1;
    }

    /// <summary>
    /// Discrete Differential Geometry (DDG) Half-Edge Mesh and Discrete Exterior Calculus (DEC) engine.
    /// Automatically welds coincident topological seams on unwelded Rhino meshes (e.g., Mesh Box, Mesh Brep)
    /// while preserving exact sharp dihedral angles on edges for Feature-Aware Heatmaps and Segmentation.
    /// </summary>
    public class DdgMesh
    {
        public readonly Mesh RawInputMesh;
        public readonly Mesh RhinoMesh;
        public readonly int VertexCount;
        public readonly int FaceCount;
        public int EdgeCount => Edges.Count;

        public readonly int[] RawToDdgVertex;
        public readonly Point3d[] Positions;
        public readonly int[][] Faces;
        public readonly Vector3d[] FaceNormals;
        public readonly double[] FaceAreas;
        public readonly Point3d[] FaceCenters;

        public readonly List<DdgHalfEdge> HalfEdges = new List<DdgHalfEdge>();
        public readonly List<DdgEdge> Edges = new List<DdgEdge>();
        public readonly List<int>[] VertexOutgoingHalfEdges;
        public readonly bool[] VertexOnBoundary;

        // Intrinsic & Differential Quantities
        public readonly double[] DualAreas;
        public readonly Vector3d[] VertexNormals;
        public readonly Vector3d[] TangentBasisX;
        public readonly Vector3d[] TangentBasisY;
        public readonly double[] GaussianCurvature;
        public readonly double[] MeanCurvature;
        public readonly double[] VertexSharpness; // max dihedral angle (radians) incident to vertex
        public readonly Vector3d[] PrincipalDir;
        public readonly double MeanEdgeLength;
        public readonly double MaxEdgeLength;

        public DdgMesh(Mesh inputMesh)
        {
            if (inputMesh == null) throw new ArgumentNullException(nameof(inputMesh));
            RawInputMesh = inputMesh;

            // Duplicate, triangulate quads, and weld coincident vertices by 3D position
            // so unwelded Rhino meshes (boxes, extrusions, Brep meshes) are topologically connected
            Mesh temp = inputMesh.DuplicateMesh();
            temp.Faces.ConvertQuadsToTriangles();
            temp.Compact();

            RhinoMesh = BuildPositionWeldedMesh(temp, out RawToDdgVertex);
            VertexCount = RhinoMesh.Vertices.Count;
            FaceCount = RhinoMesh.Faces.Count;

            Positions = new Point3d[VertexCount];
            for (int i = 0; i < VertexCount; i++)
                Positions[i] = RhinoMesh.Vertices.Point3dAt(i);

            Faces = new int[FaceCount][];
            FaceNormals = new Vector3d[FaceCount];
            FaceAreas = new double[FaceCount];
            FaceCenters = new Point3d[FaceCount];

            VertexOutgoingHalfEdges = new List<int>[VertexCount];
            for (int i = 0; i < VertexCount; i++)
                VertexOutgoingHalfEdges[i] = new List<int>(6);

            VertexOnBoundary = new bool[VertexCount];
            DualAreas = new double[VertexCount];
            VertexNormals = new Vector3d[VertexCount];
            TangentBasisX = new Vector3d[VertexCount];
            TangentBasisY = new Vector3d[VertexCount];
            GaussianCurvature = new double[VertexCount];
            MeanCurvature = new double[VertexCount];
            VertexSharpness = new double[VertexCount];
            PrincipalDir = new Vector3d[VertexCount];

            BuildHalfEdgeTopology();
            ComputeGeometricQuantities(out MeanEdgeLength, out MaxEdgeLength);
            InitializeSmoothStructureAndCurvatures();
        }

        private static Mesh BuildPositionWeldedMesh(Mesh triMesh, out int[] rawToWelded)
        {
            int rawVCount = triMesh.Vertices.Count;
            rawToWelded = new int[rawVCount];
            BoundingBox bbox = triMesh.GetBoundingBox(false);
            double diag = Math.Max(1e-6, bbox.Diagonal.Length);
            double invCell = 1e6 / diag; // spatial hash tolerance = 1e-6 * diag

            var cellToVertex = new Dictionary<(long, long, long), int>(rawVCount);
            var uniquePts = new List<Point3d>(rawVCount);

            for (int i = 0; i < rawVCount; i++)
            {
                Point3d p = triMesh.Vertices.Point3dAt(i);
                long ix = (long)Math.Round(p.X * invCell);
                long iy = (long)Math.Round(p.Y * invCell);
                long iz = (long)Math.Round(p.Z * invCell);
                var key = (ix, iy, iz);

                if (!cellToVertex.TryGetValue(key, out int weldedIdx))
                {
                    weldedIdx = uniquePts.Count;
                    uniquePts.Add(p);
                    cellToVertex[key] = weldedIdx;
                }
                rawToWelded[i] = weldedIdx;
            }

            var welded = new Mesh();
            welded.Vertices.AddVertices(uniquePts);
            for (int f = 0; f < triMesh.Faces.Count; f++)
            {
                MeshFace mf = triMesh.Faces[f];
                int a = rawToWelded[mf.A];
                int b = rawToWelded[mf.B];
                int c = rawToWelded[mf.C];
                if (a != b && b != c && c != a)
                {
                    welded.Faces.AddFace(a, b, c);
                }
            }
            welded.Normals.ComputeNormals();
            return welded;
        }

        /// <summary>
        /// Seamlessly maps a list of per-vertex scalar values to DdgMesh vertices,
        /// handling both welded DdgMesh counts and unwelded RawInputMesh counts.
        /// </summary>
        public double[] MapScalarInputToVertices(IList<double> input)
        {
            double[] res = new double[VertexCount];
            if (input == null || input.Count == 0)
            {
                // Fallback: if mesh already carries VertexColors (e.g. from SmoothScalar CM output), decode luminance!
                if (RhinoMesh.VertexColors.Count == VertexCount)
                {
                    for (int i = 0; i < VertexCount; i++)
                    {
                        var c = RhinoMesh.VertexColors[i];
                        res[i] = (0.299 * c.R + 0.587 * c.G + 0.114 * c.B) / 255.0;
                    }
                }
                return res;
            }

            if (input.Count == VertexCount)
            {
                for (int i = 0; i < VertexCount; i++) res[i] = input[i];
            }
            else if (input.Count == RawToDdgVertex.Length)
            {
                int[] counts = new int[VertexCount];
                for (int i = 0; i < RawToDdgVertex.Length; i++)
                {
                    int w = RawToDdgVertex[i];
                    res[w] += input[i];
                    counts[w]++;
                }
                for (int i = 0; i < VertexCount; i++)
                    if (counts[i] > 1) res[i] /= counts[i];
            }
            else if (input.Count == FaceCount)
            {
                int[] counts = new int[VertexCount];
                for (int f = 0; f < FaceCount; f++)
                {
                    int[] fv = Faces[f];
                    double val = input[f];
                    for (int k = 0; k < 3; k++)
                    {
                        res[fv[k]] += val;
                        counts[fv[k]]++;
                    }
                }
                for (int i = 0; i < VertexCount; i++)
                    if (counts[i] > 1) res[i] /= counts[i];
            }
            else
            {
                for (int i = 0; i < VertexCount; i++)
                    res[i] = input[Math.Min(i, input.Count - 1)];
            }
            return res;
        }

        /// <summary>
        /// Seamlessly maps a list of per-vertex or per-face vectors to DdgMesh vertices.
        /// </summary>
        public Vector3d[] MapVectorInputToVertices(IList<Vector3d> input)
        {
            Vector3d[] res = new Vector3d[VertexCount];
            if (input == null || input.Count == 0)
            {
                Array.Copy(PrincipalDir, res, VertexCount);
                return res;
            }

            if (input.Count == VertexCount)
            {
                for (int i = 0; i < VertexCount; i++) res[i] = input[i];
            }
            else if (input.Count == RawToDdgVertex.Length)
            {
                for (int i = 0; i < RawToDdgVertex.Length; i++)
                    res[RawToDdgVertex[i]] += input[i];
                for (int i = 0; i < VertexCount; i++)
                    if (res[i].Length > 1e-12) res[i].Unitize();
            }
            else if (input.Count == FaceCount)
            {
                for (int f = 0; f < FaceCount; f++)
                {
                    int[] fv = Faces[f];
                    Vector3d vf = input[f];
                    res[fv[0]] += vf;
                    res[fv[1]] += vf;
                    res[fv[2]] += vf;
                }
                for (int i = 0; i < VertexCount; i++)
                    if (res[i].Length > 1e-12) res[i].Unitize();
            }
            else
            {
                for (int i = 0; i < VertexCount; i++)
                    res[i] = input[Math.Min(i, input.Count - 1)];
            }
            return res;
        }

        private void BuildHalfEdgeTopology()
        {
            var edgeMap = new Dictionary<long, int>(FaceCount * 3);

            for (int f = 0; f < FaceCount; f++)
            {
                MeshFace mf = RhinoMesh.Faces[f];
                int v0 = mf.A, v1 = mf.B, v2 = mf.C;
                Faces[f] = new int[] { v0, v1, v2 };

                int baseHe = HalfEdges.Count;
                for (int k = 0; k < 3; k++)
                {
                    int orig = Faces[f][k];
                    int targ = Faces[f][(k + 1) % 3];
                    var he = new DdgHalfEdge
                    {
                        Index = baseHe + k,
                        OriginVertex = orig,
                        TargetVertex = targ,
                        FaceIndex = f,
                        Next = baseHe + ((k + 1) % 3),
                        Prev = baseHe + ((k + 2) % 3)
                    };
                    HalfEdges.Add(he);
                    VertexOutgoingHalfEdges[orig].Add(he.Index);

                    int minV = Math.Min(orig, targ);
                    int maxV = Math.Max(orig, targ);
                    long key = ((long)minV << 32) | (uint)maxV;

                    if (edgeMap.TryGetValue(key, out int existingEdgeIdx))
                    {
                        DdgEdge edge = Edges[existingEdgeIdx];
                        he.EdgeIndex = existingEdgeIdx;
                        if (edge.HalfEdge1 == -1)
                        {
                            edge.HalfEdge1 = he.Index;
                            HalfEdges[edge.HalfEdge0].Twin = he.Index;
                            he.Twin = edge.HalfEdge0;
                        }
                    }
                    else
                    {
                        int newEdgeIdx = Edges.Count;
                        he.EdgeIndex = newEdgeIdx;
                        var edge = new DdgEdge
                        {
                            Index = newEdgeIdx,
                            V0 = minV,
                            V1 = maxV,
                            HalfEdge0 = he.Index,
                            HalfEdge1 = -1
                        };
                        Edges.Add(edge);
                        edgeMap[key] = newEdgeIdx;
                    }
                }
            }

            foreach (var e in Edges)
            {
                if (e.IsBoundary)
                {
                    VertexOnBoundary[e.V0] = true;
                    VertexOnBoundary[e.V1] = true;
                }
            }
        }

        private void ComputeGeometricQuantities(out double meanH, out double maxH)
        {
            for (int f = 0; f < FaceCount; f++)
            {
                int i0 = Faces[f][0], i1 = Faces[f][1], i2 = Faces[f][2];
                Point3d p0 = Positions[i0], p1 = Positions[i1], p2 = Positions[i2];

                FaceCenters[f] = (p0 + p1 + p2) / 3.0;
                Vector3d cross = Vector3d.CrossProduct(p1 - p0, p2 - p0);
                double dblArea = cross.Length;
                FaceAreas[f] = Math.Max(1e-12, 0.5 * dblArea);

                if (dblArea > 1e-14)
                    FaceNormals[f] = cross / dblArea;
                else
                    FaceNormals[f] = Vector3d.ZAxis;

                double areaThird = FaceAreas[f] / 3.0;
                DualAreas[i0] += areaThird;
                DualAreas[i1] += areaThird;
                DualAreas[i2] += areaThird;

                VertexNormals[i0] += FaceNormals[f] * FaceAreas[f];
                VertexNormals[i1] += FaceNormals[f] * FaceAreas[f];
                VertexNormals[i2] += FaceNormals[f] * FaceAreas[f];

                int baseHe = f * 3;
                for (int k = 0; k < 3; k++)
                {
                    Point3d pa = Positions[Faces[f][k]];
                    Point3d pb = Positions[Faces[f][(k + 1) % 3]];
                    Point3d pc = Positions[Faces[f][(k + 2) % 3]];

                    Vector3d uOrigin1 = pb - pa;
                    Vector3d uOrigin2 = pc - pa;
                    double sinOrigin = Vector3d.CrossProduct(uOrigin1, uOrigin2).Length;
                    double cosOrigin = uOrigin1 * uOrigin2;
                    HalfEdges[baseHe + k].CornerAngle = Math.Atan2(Math.Max(1e-12, sinOrigin), cosOrigin);

                    Vector3d uOpp1 = pa - pc;
                    Vector3d uOpp2 = pb - pc;
                    double sinOpp = Math.Max(1e-10, Vector3d.CrossProduct(uOpp1, uOpp2).Length);
                    double cosOpp = uOpp1 * uOpp2;
                    double cot = cosOpp / sinOpp;
                    HalfEdges[baseHe + k].Cotan = Math.Max(-50.0, Math.Min(50.0, cot));
                }
            }

            for (int v = 0; v < VertexCount; v++)
            {
                DualAreas[v] = Math.Max(1e-12, DualAreas[v]);
                if (VertexNormals[v].Length > 1e-12)
                    VertexNormals[v].Unitize();
                else
                    VertexNormals[v] = new Vector3d(RhinoMesh.Normals[v]);
            }

            double sumLen = 0.0;
            maxH = 0.0;
            foreach (var e in Edges)
            {
                e.Length = Positions[e.V0].DistanceTo(Positions[e.V1]);
                sumLen += e.Length;
                if (e.Length > maxH) maxH = e.Length;

                double cot0 = HalfEdges[e.HalfEdge0].Cotan;
                double cot1 = e.HalfEdge1 != -1 ? HalfEdges[e.HalfEdge1].Cotan : 0.0;
                e.CotanWeight = Math.Max(1e-6, 0.5 * (cot0 + cot1));

                if (e.HalfEdge1 != -1)
                {
                    int f0 = HalfEdges[e.HalfEdge0].FaceIndex;
                    int f1 = HalfEdges[e.HalfEdge1].FaceIndex;
                    Vector3d n0 = FaceNormals[f0];
                    Vector3d n1 = FaceNormals[f1];
                    double dot = Math.Max(-1.0, Math.Min(1.0, n0 * n1));
                    e.DihedralAngle = Math.Acos(dot);

                    Vector3d centerDiff = FaceCenters[f1] - FaceCenters[f0];
                    double sign = (centerDiff * (n0 - n1)) >= 0.0 ? 1.0 : -1.0;
                    e.SignedDihedralAngle = sign * e.DihedralAngle;

                    if (e.DihedralAngle > VertexSharpness[e.V0]) VertexSharpness[e.V0] = e.DihedralAngle;
                    if (e.DihedralAngle > VertexSharpness[e.V1]) VertexSharpness[e.V1] = e.DihedralAngle;
                }
                else
                {
                    e.DihedralAngle = 0.0;
                    e.SignedDihedralAngle = 0.0;
                }
            }
            meanH = Edges.Count > 0 ? sumLen / Edges.Count : 1.0;
        }

        private void InitializeSmoothStructureAndCurvatures()
        {
            Vector3d[] laplacePos = new Vector3d[VertexCount];

            for (int v = 0; v < VertexCount; v++)
            {
                Vector3d n = VertexNormals[v];
                var outHes = VertexOutgoingHalfEdges[v];
                if (outHes.Count == 0)
                {
                    TangentBasisX[v] = Vector3d.XAxis;
                    TangentBasisY[v] = Vector3d.YAxis;
                    continue;
                }

                Vector3d e0 = Positions[HalfEdges[outHes[0]].TargetVertex] - Positions[v];
                Vector3d bx = e0 - (e0 * n) * n;
                if (bx.Length < 1e-12)
                {
                    bx = Vector3d.CrossProduct(n, Math.Abs(n.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                }
                bx.Unitize();
                Vector3d by = Vector3d.CrossProduct(n, bx);
                by.Unitize();
                TangentBasisX[v] = bx;
                TangentBasisY[v] = by;

                double angleSum = 0.0;
                for (int k = 0; k < outHes.Count; k++)
                {
                    var he = HalfEdges[outHes[k]];
                    angleSum += he.CornerAngle;
                    Vector3d vec = Positions[he.TargetVertex] - Positions[v];
                    double x = vec * bx;
                    double y = vec * by;
                    double ang = Math.Atan2(y, x);
                    if (ang < 0) ang += 2.0 * Math.PI;
                    he.AngularCoordinate = ang;
                }

                GaussianCurvature[v] = (VertexOnBoundary[v] ? Math.PI : 2.0 * Math.PI) - angleSum;
            }

            foreach (var e in Edges)
            {
                int i = e.V0;
                int j = e.V1;
                double w = e.CotanWeight;
                Vector3d diff = Positions[j] - Positions[i];
                laplacePos[i] += w * diff;
                laplacePos[j] -= w * diff;
            }

            for (int v = 0; v < VertexCount; v++)
            {
                Vector3d hn = laplacePos[v] / (2.0 * DualAreas[v]);
                double sign = (hn * VertexNormals[v]) >= 0 ? 1.0 : -1.0;
                MeanCurvature[v] = sign * hn.Length;

                double c2x = 0.0, c2y = 0.0;
                foreach (int heIdx in VertexOutgoingHalfEdges[v])
                {
                    var he = HalfEdges[heIdx];
                    if (he.Twin == -1) continue;
                    Vector3d nf0 = FaceNormals[he.FaceIndex];
                    Vector3d nf1 = FaceNormals[HalfEdges[he.Twin].FaceIndex];
                    Vector3d edgeVec = Positions[he.TargetVertex] - Positions[v];
                    double len = edgeVec.Length;
                    if (len < 1e-12) continue;
                    edgeVec /= len;

                    double sinPhi = Vector3d.CrossProduct(nf0, nf1) * edgeVec;
                    double cosPhi = nf0 * nf1;
                    double dihedral = Math.Atan2(sinPhi, cosPhi);
                    double weight = len * dihedral;

                    double theta = he.AngularCoordinate;
                    c2x += weight * Math.Cos(2.0 * theta);
                    c2y += weight * Math.Sin(2.0 * theta);
                }
                double halfAng = 0.5 * Math.Atan2(c2y, c2x);
                PrincipalDir[v] = Math.Cos(halfAng) * TangentBasisX[v] + Math.Sin(halfAng) * TangentBasisY[v];
            }
        }

        public SparseMatrixBuilder BuildCotanLaplacianBuilder(double scale = 1.0)
        {
            var builder = new SparseMatrixBuilder(VertexCount, VertexCount);
            foreach (var e in Edges)
            {
                double w = scale * e.CotanWeight;
                builder.Add(e.V0, e.V0, w);
                builder.Add(e.V1, e.V1, w);
                builder.Add(e.V0, e.V1, -w);
                builder.Add(e.V1, e.V0, -w);
            }
            return builder;
        }

        /// <summary>
        /// Builds an Anisotropic Feature-Barrier Cotan-Laplacian where strong dihedral creases
        /// and vertex normal discontinuities act as thermal insulators / diffusion barriers.
        /// </summary>
        public SparseMatrixBuilder BuildFeatureLaplacianBuilder(double scale, double barrierStrength)
        {
            var builder = new SparseMatrixBuilder(VertexCount, VertexCount);
            double beta = Math.Max(0.0, barrierStrength);
            foreach (var e in Edges)
            {
                // Combine face dihedral crease angle + vertex normal difference
                double normalDiff = 1.0 - Math.Max(-1.0, Math.Min(1.0, VertexNormals[e.V0] * VertexNormals[e.V1]));
                double edgeCrease = 1.0 - Math.Cos(e.DihedralAngle);
                // Concave creases get 1.5x stronger barrier (Katz & Tal / Variational Cuts rule)
                double concaveBoost = e.SignedDihedralAngle < -0.05 ? 1.5 : 1.0;

                double attenuation = Math.Exp(-beta * concaveBoost * (4.0 * edgeCrease + 2.0 * normalDiff));
                double w = scale * e.CotanWeight * Math.Max(1e-6, attenuation);

                builder.Add(e.V0, e.V0, w);
                builder.Add(e.V1, e.V1, w);
                builder.Add(e.V0, e.V1, -w);
                builder.Add(e.V1, e.V0, -w);
            }
            return builder;
        }

        public void ComputeGradient(double[] u, out Vector3d[] faceGradients, out Vector3d[] vertexGradients)
        {
            faceGradients = new Vector3d[FaceCount];
            vertexGradients = new Vector3d[VertexCount];

            for (int f = 0; f < FaceCount; f++)
            {
                int i0 = Faces[f][0], i1 = Faces[f][1], i2 = Faces[f][2];
                Point3d p0 = Positions[i0], p1 = Positions[i1], p2 = Positions[i2];
                Vector3d n = FaceNormals[f];

                Vector3d e0 = p2 - p1;
                Vector3d e1 = p0 - p2;
                Vector3d e2 = p1 - p0;

                Vector3d grad = (u[i0] * Vector3d.CrossProduct(n, e0) +
                                 u[i1] * Vector3d.CrossProduct(n, e1) +
                                 u[i2] * Vector3d.CrossProduct(n, e2)) / (2.0 * FaceAreas[f]);

                faceGradients[f] = grad;
                double w = FaceAreas[f];
                vertexGradients[i0] += grad * w;
                vertexGradients[i1] += grad * w;
                vertexGradients[i2] += grad * w;
            }

            for (int v = 0; v < VertexCount; v++)
                vertexGradients[v] /= (3.0 * DualAreas[v]);
        }

        public double[] ComputeIntegratedDivergence(Vector3d[] faceVectors)
        {
            double[] div = new double[VertexCount];
            for (int f = 0; f < FaceCount; f++)
            {
                Vector3d X = faceVectors[f];
                int baseHe = f * 3;
                for (int k = 0; k < 3; k++)
                {
                    var heOut = HalfEdges[baseHe + k];
                    var heIn = HalfEdges[baseHe + ((k + 2) % 3)];
                    int i = heOut.OriginVertex;
                    int j = heOut.TargetVertex;
                    int m = heIn.OriginVertex;

                    Vector3d e1 = Positions[j] - Positions[i];
                    Vector3d e2 = Positions[m] - Positions[i];

                    div[i] += 0.5 * (heOut.Cotan * (e1 * X) + heIn.Cotan * (e2 * X));
                }
            }
            return div;
        }

        public int FindClosestVertex(Point3d pt)
        {
            int best = 0;
            double minDistSq = double.MaxValue;
            for (int i = 0; i < VertexCount; i++)
            {
                double dSq = Positions[i].DistanceToSquared(pt);
                if (dSq < minDistSq)
                {
                    minDistSq = dSq;
                    best = i;
                }
            }
            return best;
        }

        public List<Polyline> ExtractIsoContours(double[] vertexScalar, IEnumerable<double> isoValues)
        {
            var polylines = new List<Polyline>();
            foreach (double iso in isoValues)
            {
                var segments = new List<Line>();
                for (int f = 0; f < FaceCount; f++)
                {
                    int i0 = Faces[f][0], i1 = Faces[f][1], i2 = Faces[f][2];
                    double v0 = vertexScalar[i0] - iso;
                    double v1 = vertexScalar[i1] - iso;
                    double v2 = vertexScalar[i2] - iso;

                    var pts = new List<Point3d>(2);
                    TryEdgeCross(Positions[i0], Positions[i1], v0, v1, pts);
                    TryEdgeCross(Positions[i1], Positions[i2], v1, v2, pts);
                    TryEdgeCross(Positions[i2], Positions[i0], v2, v0, pts);

                    if (pts.Count >= 2 && pts[0].DistanceToSquared(pts[1]) > 1e-14)
                        segments.Add(new Line(pts[0], pts[1]));
                }

                if (segments.Count > 0)
                {
                    Curve[] joined = Curve.JoinCurves(segments.ConvertAll(s => (Curve)new LineCurve(s)), MeanEdgeLength * 1e-3);
                    if (joined != null)
                    {
                        foreach (var crv in joined)
                        {
                            if (crv.TryGetPolyline(out Polyline pl))
                                polylines.Add(pl);
                        }
                    }
                }
            }
            return polylines;
        }

        private static void TryEdgeCross(Point3d pA, Point3d pB, double vA, double vB, List<Point3d> pts)
        {
            if ((vA < 0.0 && vB >= 0.0) || (vB < 0.0 && vA >= 0.0))
            {
                double denom = vA - vB;
                double t = Math.Abs(denom) > 1e-14 ? vA / denom : 0.5;
                pts.Add(pA + t * (pB - pA));
            }
        }
    }
}
