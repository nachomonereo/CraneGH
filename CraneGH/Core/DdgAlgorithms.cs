using System;
using System.Collections.Generic;
using System.Drawing;
using Rhino.Geometry;

namespace CraneGH.Core
{
    public static partial class DdgAlgorithms
    {
        // =========================================================================================
        // 1. THE HEAT METHOD FOR DISTANCE COMPUTATION (Crane, Weischedel, Wardetzky 2013)
        // =========================================================================================
        public static double[] ComputeHeatDistance(
            DdgMesh mesh,
            IEnumerable<int> sourceVertices,
            double timeFactor,
            out Vector3d[] normalizedFaceGradients,
            out Vector3d[] vertexGradients)
        {
            int n = mesh.VertexCount;
            double h = mesh.MeanEdgeLength;
            double t = Math.Max(1e-6, timeFactor) * h * h;

            var heatBuilder = mesh.BuildCotanLaplacianBuilder(t);
            for (int i = 0; i < n; i++)
                heatBuilder.Add(i, i, mesh.DualAreas[i]);

            double[] rhs = new double[n];
            bool anySource = false;
            foreach (int src in sourceVertices)
            {
                if (src >= 0 && src < n)
                {
                    rhs[src] = 1.0;
                    anySource = true;
                }
            }
            if (!anySource && n > 0) rhs[0] = 1.0;

            CsrMatrix heatMat = heatBuilder.Build();
            double[] u = SparseLinearSolver.SolvePCG(heatMat, rhs, 1200, 1e-8);

            mesh.ComputeGradient(u, out Vector3d[] gradU, out _);
            normalizedFaceGradients = new Vector3d[mesh.FaceCount];
            for (int f = 0; f < mesh.FaceCount; f++)
            {
                Vector3d g = -gradU[f];
                if (g.Length > 1e-14)
                    g.Unitize();
                normalizedFaceGradients[f] = g;
            }

            double[] divX = mesh.ComputeIntegratedDivergence(normalizedFaceGradients);
            var lapBuilder = mesh.BuildCotanLaplacianBuilder(1.0);
            lapBuilder.ShiftDiagonal(1e-9);
            CsrMatrix lapMat = lapBuilder.Build();

            double[] phi = SparseLinearSolver.SolvePCG(lapMat, divX, 1500, 1e-8);

            double minPhi = double.MaxValue;
            for (int i = 0; i < n; i++)
                if (phi[i] < minPhi) minPhi = phi[i];
            for (int i = 0; i < n; i++)
                phi[i] -= minPhi;

            mesh.ComputeGradient(phi, out _, out vertexGradients);
            for (int i = 0; i < n; i++)
            {
                if (vertexGradients[i].Length > 1e-12)
                    vertexGradients[i].Unitize();
            }

            return phi;
        }

        // =========================================================================================
        // 2. THE VECTOR HEAT METHOD: PARALLEL TRANSPORT & LOG MAP (Sharp, Soliman, Crane 2019)
        // =========================================================================================
        public static Vector3d[] ComputeVectorHeatTransport(
            DdgMesh mesh,
            IList<int> sourceVertices,
            IList<Vector3d> sourceVectors,
            double timeFactor = 1.0)
        {
            int n = mesh.VertexCount;
            double h = mesh.MeanEdgeLength;
            double t = Math.Max(1e-5, timeFactor) * h * h;

            var connBuilder = new SparseMatrixBuilder(2 * n, 2 * n);
            for (int i = 0; i < n; i++)
            {
                connBuilder.Add(2 * i, 2 * i, mesh.DualAreas[i]);
                connBuilder.Add(2 * i + 1, 2 * i + 1, mesh.DualAreas[i]);
            }

            foreach (var e in mesh.Edges)
            {
                int i = e.V0;
                int j = e.V1;
                double w = t * e.CotanWeight;

                double thetaI = FindHalfEdgeAngle(mesh, i, j);
                double thetaJ = FindHalfEdgeAngle(mesh, j, i);
                double phi = thetaJ + Math.PI - thetaI;
                double c = Math.Cos(phi);
                double s = Math.Sin(phi);

                connBuilder.Add(2 * i, 2 * i, w);
                connBuilder.Add(2 * i + 1, 2 * i + 1, w);
                connBuilder.Add(2 * j, 2 * j, w);
                connBuilder.Add(2 * j + 1, 2 * j + 1, w);

                connBuilder.Add(2 * i, 2 * j, -w * c);
                connBuilder.Add(2 * i, 2 * j + 1, -w * s);
                connBuilder.Add(2 * i + 1, 2 * j, w * s);
                connBuilder.Add(2 * i + 1, 2 * j + 1, -w * c);

                connBuilder.Add(2 * j, 2 * i, -w * c);
                connBuilder.Add(2 * j, 2 * i + 1, w * s);
                connBuilder.Add(2 * j + 1, 2 * i, -w * s);
                connBuilder.Add(2 * j + 1, 2 * i + 1, -w * c);
            }

            double[] rhsVec = new double[2 * n];
            double[] rhsMag = new double[n];
            double[] rhsInd = new double[n];

            for (int k = 0; k < sourceVertices.Count; k++)
            {
                int v = sourceVertices[k];
                if (v < 0 || v >= n) continue;
                Vector3d vec = sourceVectors[Math.Min(k, sourceVectors.Count - 1)];
                double vx = vec * mesh.TangentBasisX[v];
                double vy = vec * mesh.TangentBasisY[v];
                double mag = Math.Sqrt(vx * vx + vy * vy);
                if (mag > 1e-12)
                {
                    rhsVec[2 * v] += vx / mag;
                    rhsVec[2 * v + 1] += vy / mag;
                    rhsMag[v] += mag;
                    rhsInd[v] += 1.0;
                }
            }

            CsrMatrix connMat = connBuilder.Build();
            double[] uVec = SparseLinearSolver.SolvePCG(connMat, rhsVec, 1000, 1e-7);

            var scalarHeatBuilder = mesh.BuildCotanLaplacianBuilder(t);
            for (int i = 0; i < n; i++)
                scalarHeatBuilder.Add(i, i, mesh.DualAreas[i]);
            CsrMatrix scalarHeatMat = scalarHeatBuilder.Build();

            double[] uMag = SparseLinearSolver.SolvePCG(scalarHeatMat, rhsMag, 800, 1e-7);
            double[] uInd = SparseLinearSolver.SolvePCG(scalarHeatMat, rhsInd, 800, 1e-7);

            Vector3d[] result = new Vector3d[n];
            for (int i = 0; i < n; i++)
            {
                double ux = uVec[2 * i];
                double uy = uVec[2 * i + 1];
                double norm = Math.Sqrt(ux * ux + uy * uy);
                double interpMag = uInd[i] > 1e-12 ? uMag[i] / uInd[i] : 1.0;
                if (norm > 1e-12)
                {
                    ux = (ux / norm) * interpMag;
                    uy = (uy / norm) * interpMag;
                }
                result[i] = ux * mesh.TangentBasisX[i] + uy * mesh.TangentBasisY[i];
            }
            return result;
        }

        public static Point3d[] ComputeLogarithmicMap(
            DdgMesh mesh,
            int originVertex,
            Vector3d refDirection,
            out double[] radii,
            out double[] angles)
        {
            int n = mesh.VertexCount;
            radii = ComputeHeatDistance(mesh, new int[] { originVertex }, 1.0, out _, out Vector3d[] radialGrad);

            Vector3d x0 = refDirection - (refDirection * mesh.VertexNormals[originVertex]) * mesh.VertexNormals[originVertex];
            if (x0.Length < 1e-12)
                x0 = mesh.TangentBasisX[originVertex];
            x0.Unitize();
            Vector3d y0 = Vector3d.CrossProduct(mesh.VertexNormals[originVertex], x0);
            y0.Unitize();

            Vector3d[] transX = ComputeVectorHeatTransport(mesh, new int[] { originVertex }, new Vector3d[] { x0 }, 1.0);
            Vector3d[] transY = ComputeVectorHeatTransport(mesh, new int[] { originVertex }, new Vector3d[] { y0 }, 1.0);

            Point3d[] uv = new Point3d[n];
            angles = new double[n];
            for (int i = 0; i < n; i++)
            {
                if (i == originVertex)
                {
                    uv[i] = Point3d.Origin;
                    angles[i] = 0.0;
                    continue;
                }
                Vector3d rad = radialGrad[i];
                double cx = rad * transX[i];
                double cy = rad * transY[i];
                double theta = Math.Atan2(cy, cx);
                angles[i] = theta;
                uv[i] = new Point3d(radii[i] * Math.Cos(theta), radii[i] * Math.Sin(theta), 0.0);
            }
            return uv;
        }

        // =========================================================================================
        // 3. GLOBALLY OPTIMAL DIRECTION FIELDS (Knöppel, Crane, Pinkall, Schröder 2013)
        // =========================================================================================
        public static Vector3d[] ComputeOptimalDirectionField(
            DdgMesh mesh,
            int symmetryDegree,
            double curvatureAlignment,
            out Vector3d[] orthoField,
            out List<Point3d> singularities)
        {
            int n = mesh.VertexCount;
            int kSym = Math.Max(1, symmetryDegree);

            var builder = new SparseMatrixBuilder(2 * n, 2 * n);
            foreach (var e in mesh.Edges)
            {
                int i = e.V0;
                int j = e.V1;
                double w = e.CotanWeight;

                double thetaI = FindHalfEdgeAngle(mesh, i, j);
                double thetaJ = FindHalfEdgeAngle(mesh, j, i);
                double phi = kSym * (thetaI - thetaJ + Math.PI);
                double cosP = Math.Cos(phi);
                double sinP = Math.Sin(phi);

                builder.Add(2 * i, 2 * i, w);
                builder.Add(2 * i + 1, 2 * i + 1, w);
                builder.Add(2 * j, 2 * j, w);
                builder.Add(2 * j + 1, 2 * j + 1, w);

                builder.Add(2 * i, 2 * j, -w * cosP);
                builder.Add(2 * i, 2 * j + 1, w * sinP);
                builder.Add(2 * i + 1, 2 * j, -w * sinP);
                builder.Add(2 * i + 1, 2 * j + 1, -w * cosP);

                builder.Add(2 * j, 2 * i, -w * cosP);
                builder.Add(2 * j, 2 * i + 1, -w * sinP);
                builder.Add(2 * j + 1, 2 * i, w * sinP);
                builder.Add(2 * j + 1, 2 * i + 1, -w * cosP);
            }

            builder.ShiftDiagonal(1e-4);

            double[] solution;
            if (curvatureAlignment > 1e-4)
            {
                double alpha = curvatureAlignment;
                double[] rhs = new double[2 * n];
                for (int i = 0; i < n; i++)
                {
                    double mass = mesh.DualAreas[i];
                    builder.Add(2 * i, 2 * i, alpha * mass);
                    builder.Add(2 * i + 1, 2 * i + 1, alpha * mass);

                    Vector3d pDir = mesh.PrincipalDir[i];
                    double px = pDir * mesh.TangentBasisX[i];
                    double py = pDir * mesh.TangentBasisY[i];
                    double ang = Math.Atan2(py, px) * kSym;
                    double mag = Math.Abs(mesh.MeanCurvature[i]) + 1e-3;
                    rhs[2 * i] = alpha * mass * mag * Math.Cos(ang);
                    rhs[2 * i + 1] = alpha * mass * mag * Math.Sin(ang);
                }
                CsrMatrix A = builder.Build();
                solution = SparseLinearSolver.SolvePCG(A, rhs, 1000, 1e-7);
            }
            else
            {
                double[] massDiag = new double[2 * n];
                for (int i = 0; i < n; i++)
                {
                    massDiag[2 * i] = mesh.DualAreas[i];
                    massDiag[2 * i + 1] = mesh.DualAreas[i];
                }
                CsrMatrix A = builder.Build();
                solution = SparseLinearSolver.SmallestEigenvector(A, massDiag, 12, 500);
            }

            Vector3d[] field = new Vector3d[n];
            orthoField = new Vector3d[n];
            double[] rootAngles = new double[n];

            for (int i = 0; i < n; i++)
            {
                double re = solution[2 * i];
                double im = solution[2 * i + 1];
                double ang = Math.Atan2(im, re) / kSym;
                rootAngles[i] = ang;
                field[i] = Math.Cos(ang) * mesh.TangentBasisX[i] + Math.Sin(ang) * mesh.TangentBasisY[i];
                orthoField[i] = -Math.Sin(ang) * mesh.TangentBasisX[i] + Math.Cos(ang) * mesh.TangentBasisY[i];
            }

            singularities = new List<Point3d>();
            for (int f = 0; f < mesh.FaceCount; f++)
            {
                int baseHe = f * 3;
                double totalDelta = 0.0;
                for (int c = 0; c < 3; c++)
                {
                    var he = mesh.HalfEdges[baseHe + c];
                    int i = he.OriginVertex;
                    int j = he.TargetVertex;
                    double thetaI = he.AngularCoordinate;
                    double thetaJ = FindHalfEdgeAngle(mesh, j, i);
                    double dTheta = thetaJ + Math.PI - thetaI;
                    double diff = kSym * (rootAngles[j] - (rootAngles[i] + dTheta));
                    diff = WrapPi(diff);
                    totalDelta += diff;
                }
                if (Math.Abs(totalDelta) > 1.5)
                    singularities.Add(mesh.FaceCenters[f]);
            }

            return field;
        }

        // =========================================================================================
        // 4. TRIVIAL CONNECTIONS ON DISCRETE SURFACES (Crane, Desbrun, Schröder 2010)
        //    Exact Primal/Dual parallel transport from StripePatterns/Mesh.cpp & extract_direction_field.m
        // =========================================================================================
        public static Vector3d[] ComputeTrivialConnections(
            DdgMesh mesh,
            IList<int> singularityVertices,
            IList<double> singularityIndices,
            double globalAngleOffset,
            out Vector3d[] vertexField)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;

            // Build unweighted Primal Laplacian (d0^T * d0) so omega_ij = u_j - u_i satisfies d0^T omega = -K + 2*pi*S
            var lapBuilder = new SparseMatrixBuilder(nV, nV);
            foreach (var e in mesh.Edges)
            {
                lapBuilder.Add(e.V0, e.V0, 1.0);
                lapBuilder.Add(e.V1, e.V1, 1.0);
                lapBuilder.Add(e.V0, e.V1, -1.0);
                lapBuilder.Add(e.V1, e.V0, -1.0);
            }
            lapBuilder.ShiftDiagonal(1e-8);

            double[] rhs = new double[nV];
            for (int i = 0; i < nV; i++)
                rhs[i] = -mesh.GaussianCurvature[i];

            if (singularityVertices != null && singularityVertices.Count > 0 && singularityIndices != null && singularityIndices.Count > 0)
            {
                int count = Math.Min(singularityVertices.Count, singularityIndices.Count);
                for (int k = 0; k < count; k++)
                {
                    int vIdx = singularityVertices[k];
                    if (vIdx >= 0 && vIdx < nV)
                        rhs[vIdx] += 2.0 * Math.PI * singularityIndices[k];
                }
            }
            else
            {
                // Default: place compensating singularity at vertex of highest Gaussian curvature magnitude
                int maxKVert = 0;
                for (int i = 1; i < nV; i++)
                    if (Math.Abs(mesh.GaussianCurvature[i]) > Math.Abs(mesh.GaussianCurvature[maxKVert]))
                        maxKVert = i;
                double totalK = 0.0;
                for (int i = 0; i < nV; i++) totalK += mesh.GaussianCurvature[i];
                rhs[maxKVert] += totalK;
            }

            // Enforce Gauss-Bonnet zero-mean solvability condition
            double sumRhs = 0.0;
            for (int i = 0; i < nV; i++) sumRhs += rhs[i];
            double meanCorr = sumRhs / nV;
            for (int i = 0; i < nV; i++) rhs[i] -= meanCorr;

            double[] u = SparseLinearSolver.SolvePCG(lapBuilder.Build(), rhs, 1500, 1e-8);

            // Parallel transport across vertices via BFS (Lines 335-373 of StripePatterns/Mesh.cpp)
            vertexField = new Vector3d[nV];
            bool[] visited = new bool[nV];
            double[] vertAngles = new double[nV];

            var queue = new Queue<int>();
            vertAngles[0] = globalAngleOffset;
            visited[0] = true;
            queue.Enqueue(0);

            while (queue.Count > 0)
            {
                int vi = queue.Dequeue();
                foreach (int heIdx in mesh.VertexOutgoingHalfEdges[vi])
                {
                    var he = mesh.HalfEdges[heIdx];
                    int vj = he.TargetVertex;
                    if (!visited[vj])
                    {
                        double thetaI = he.AngularCoordinate;
                        double thetaJ = FindHalfEdgeAngle(mesh, vj, vi) + Math.PI;
                        double dTheta = thetaJ - thetaI;
                        double omegaIJ = u[vj] - u[vi];
                        vertAngles[vj] = WrapPi(vertAngles[vi] + dTheta - omegaIJ);
                        visited[vj] = true;
                        queue.Enqueue(vj);
                    }
                }
            }

            for (int i = 0; i < nV; i++)
            {
                vertexField[i] = Math.Cos(vertAngles[i]) * mesh.TangentBasisX[i] +
                                 Math.Sin(vertAngles[i]) * mesh.TangentBasisY[i];
            }

            Vector3d[] faceField = new Vector3d[nF];
            for (int f = 0; f < nF; f++)
            {
                int i0 = mesh.Faces[f][0], i1 = mesh.Faces[f][1], i2 = mesh.Faces[f][2];
                Vector3d avg = vertexField[i0] + vertexField[i1] + vertexField[i2];
                avg -= (avg * mesh.FaceNormals[f]) * mesh.FaceNormals[f];
                if (avg.Length > 1e-12) avg.Unitize();
                faceField[f] = avg;
            }

            return faceField;
        }

        // =========================================================================================
        // 5. STRIPE PATTERNS WITH EXACT DISTANCE CONTROL & GUIDE CURVES (Knöppel, Crane et al. 2015)
        //    Includes exact per-triangle corner unwrapping (assignTextureCoordinates) so equidistant
        //    curves are extracted cleanly at exact spacing D even across sheet flips & bifurcations!
        // =========================================================================================
        public static double[] ComputeStripePatternsByDistance(
            DdgMesh mesh,
            IList<Vector3d> inputDirectionField,
            IList<Curve> guideCurves,
            double stripeSpacingDistance,
            int symmetryDegree,
            out List<Polyline> equidistantStripeCurves,
            out List<Point3d> branchPoints)
        {
            int nV = mesh.VertexCount;
            double spacing = stripeSpacingDistance > 1e-6 ? stripeSpacingDistance : 2.5 * mesh.MeanEdgeLength;
            // Angular wavenumber lambda in radians per Rhino unit so 1 period (2*pi) = spacing
            double lambda = (2.0 * Math.PI) / spacing;

            // 1. Determine tangent direction field phiDir[v]
            var pinnedPhaseVertices = new HashSet<int>();
            Vector3d[] dirField;

            if (guideCurves != null && guideCurves.Count > 0)
            {
                // Extract tangent vectors along guide curves, rotate 90 degrees in tangent plane
                // so stripe iso-curves run PARALLEL to the user's guide curves!
                var srcVerts = new List<int>();
                var srcVecs = new List<Vector3d>();
                foreach (Curve c in guideCurves)
                {
                    if (c == null) continue;
                    int sampleCount = Math.Max(16, (int)(c.GetLength() / (0.5 * mesh.MeanEdgeLength)));
                    sampleCount = Math.Min(300, sampleCount);
                    c.DivideByCount(sampleCount, true, out Point3d[] pts);
                    if (pts == null) continue;

                    foreach (Point3d p in pts)
                    {
                        int vIdx = mesh.FindClosestVertex(p);
                        if (c.ClosestPoint(p, out double tParam))
                        {
                            Vector3d curveTan = c.TangentAt(tParam);
                            // Normal in surface tangent plane (perpendicular to curve so stripes are parallel to curve)
                            Vector3d stripeGradDir = Vector3d.CrossProduct(mesh.VertexNormals[vIdx], curveTan);
                            if (stripeGradDir.Length > 1e-12)
                            {
                                stripeGradDir.Unitize();
                                srcVerts.Add(vIdx);
                                srcVecs.Add(stripeGradDir);
                                pinnedPhaseVertices.Add(vIdx);
                            }
                        }
                    }
                }
                dirField = srcVerts.Count > 0
                    ? ComputeVectorHeatTransport(mesh, srcVerts, srcVecs, 2.0)
                    : mesh.PrincipalDir;
            }
            else if (inputDirectionField != null && inputDirectionField.Count > 0)
            {
                Vector3d[] mapped = mesh.MapVectorInputToVertices(inputDirectionField);
                // Rotate by 90 degrees in tangent plane so stripes align WITH the input vectors
                dirField = new Vector3d[nV];
                for (int i = 0; i < nV; i++)
                {
                    Vector3d ortho = Vector3d.CrossProduct(mesh.VertexNormals[i], mapped[i]);
                    dirField[i] = ortho.Length > 1e-12 ? ortho : mesh.PrincipalDir[i];
                }
            }
            else
            {
                dirField = ComputeOptimalDirectionField(mesh, 2, 0.2, out _, out _);
            }

            double[] phiDir = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                Vector3d v = dirField[i];
                double x = v * mesh.TangentBasisX[i];
                double y = v * mesh.TangentBasisY[i];
                phiDir[i] = Math.Atan2(y, x);
            }

            // 2. Build 2|V| x 2|V| Stripe Energy Matrix A (Mesh::buildEnergy in StripePatterns/Mesh.cpp)
            var builder = new SparseMatrixBuilder(2 * nV, 2 * nV);
            double[] edgeOmega = new double[mesh.EdgeCount];
            bool[] edgeCrossesSheets = new bool[mesh.EdgeCount];

            foreach (var e in mesh.Edges)
            {
                int vi = e.V0;
                int vj = e.V1;
                double w = e.CotanWeight;

                double thetaI = FindHalfEdgeAngle(mesh, vi, vj);
                double thetaJ = FindHalfEdgeAngle(mesh, vj, vi) + Math.PI;
                double dTheta = thetaJ - thetaI;

                double transportedPhiI = phiDir[vi] + dTheta;
                double dot = Math.Cos(transportedPhiI - phiDir[vj]);
                double s = (symmetryDegree == 1 || dot >= 0.0) ? 1.0 : -1.0;
                edgeCrossesSheets[e.Index] = (s < 0.0);

                double phiI = phiDir[vi];
                double phiJ = s > 0.0 ? phiDir[vj] : WrapPi(phiDir[vj] + Math.PI);

                double omegaIJ = lambda * 0.5 * e.Length * (Math.Cos(phiI - thetaI) + Math.Cos(phiJ - thetaJ));
                edgeOmega[e.Index] = omegaIJ;

                double a = w * Math.Cos(omegaIJ);
                double b = w * Math.Sin(omegaIJ);

                int i = 2 * vi;
                int j = 2 * vj;

                builder.Add(i, i, w);
                builder.Add(i + 1, i + 1, w);
                builder.Add(j, j, w);
                builder.Add(j + 1, j + 1, w);

                if (s > 0.0)
                {
                    builder.Add(i, j, -a);     builder.Add(i, j + 1, -b);
                    builder.Add(i + 1, j, b);  builder.Add(i + 1, j + 1, -a);

                    builder.Add(j, i, -a);     builder.Add(j, i + 1, b);
                    builder.Add(j + 1, i, -b); builder.Add(j + 1, i + 1, -a);
                }
                else
                {
                    builder.Add(i, j, -a);     builder.Add(i, j + 1, b);
                    builder.Add(i + 1, j, b);  builder.Add(i + 1, j + 1, a);

                    builder.Add(j, i, -a);     builder.Add(j, i + 1, b);
                    builder.Add(j + 1, i, b);  builder.Add(j + 1, i + 1, a);
                }
            }

            builder.ShiftDiagonal(1e-4);

            double[] groundState;
            if (pinnedPhaseVertices.Count > 0)
            {
                // Soft-pin psi = (1, 0) along guide curves so equidistant stripes start right on the guide curves!
                double[] rhs = new double[2 * nV];
                const double pinWeight = 100.0;
                foreach (int pv in pinnedPhaseVertices)
                {
                    builder.Add(2 * pv, 2 * pv, pinWeight);
                    builder.Add(2 * pv + 1, 2 * pv + 1, pinWeight);
                    rhs[2 * pv] = pinWeight * 1.0;
                    rhs[2 * pv + 1] = 0.0;
                }
                groundState = SparseLinearSolver.SolvePCG(builder.Build(), rhs, 1200, 1e-7);
            }
            else
            {
                double[] massDiag = new double[2 * nV];
                for (int i = 0; i < nV; i++)
                {
                    massDiag[2 * i] = mesh.DualAreas[i];
                    massDiag[2 * i + 1] = mesh.DualAreas[i];
                }
                groundState = SparseLinearSolver.SmallestEigenvector(builder.Build(), massDiag, 14, 600);
            }

            double[] psiRe = new double[nV];
            double[] psiIm = new double[nV];
            double[] stripeScalar = new double[nV];

            for (int i = 0; i < nV; i++)
            {
                double re = groundState[2 * i];
                double im = groundState[2 * i + 1];
                double norm = Math.Sqrt(re * re + im * im);
                if (norm > 1e-12) { re /= norm; im /= norm; }
                else { re = 1.0; im = 0.0; }
                psiRe[i] = re;
                psiIm[i] = im;
                stripeScalar[i] = re; // cos(alpha)
            }

            // 3. Exact Per-Triangle Corner Unwrapping (Mesh::assignTextureCoordinates, StripePatterns/Mesh.cpp lines 677-741)
            //    and extraction of equidistant curves at every 2*pi period!
            branchPoints = new List<Point3d>();
            var stripeSegments = new List<Line>();

            for (int f = 0; f < mesh.FaceCount; f++)
            {
                var hij = mesh.HalfEdges[f * 3 + 0];
                var hjk = mesh.HalfEdges[f * 3 + 1];
                var hki = mesh.HalfEdges[f * 3 + 2];

                int vi = hij.OriginVertex;
                int vj = hjk.OriginVertex;
                int vk = hki.OriginVertex;

                double pI_re = psiRe[vi], pI_im = psiIm[vi];
                double pJ_re = psiRe[vj], pJ_im = psiIm[vj];
                double pK_re = psiRe[vk], pK_im = psiIm[vk];

                var eIJ = mesh.Edges[hij.EdgeIndex];
                var eJK = mesh.Edges[hjk.EdgeIndex];
                var eKI = mesh.Edges[hki.EdgeIndex];

                double cIJ = (eIJ.V0 == vi) ? 1.0 : -1.0;
                double cJK = (eJK.V0 == vj) ? 1.0 : -1.0;
                double cKI = (eKI.V0 == vk) ? 1.0 : -1.0;

                double omegaIJ = cIJ * edgeOmega[eIJ.Index];
                double omegaJK = cJK * edgeOmega[eJK.Index];
                double omegaKI = cKI * edgeOmega[eKI.Index];

                if (edgeCrossesSheets[eIJ.Index])
                {
                    pJ_im = -pJ_im;
                    omegaIJ = cIJ * omegaIJ;
                    omegaJK = -cJK * omegaJK;
                }
                if (edgeCrossesSheets[eKI.Index])
                {
                    pK_im = -pK_im;
                    omegaKI = -cKI * omegaKI;
                    omegaJK = cJK * omegaJK;
                }

                double alphaI = Math.Atan2(pI_im, pI_re);
                double alphaJ = alphaI + omegaIJ - ComplexDivArg(omegaIJ, pI_re, pI_im, pJ_re, pJ_im);
                double alphaK = alphaJ + omegaJK - ComplexDivArg(omegaJK, pJ_re, pJ_im, pK_re, pK_im);
                double alphaL = alphaK + omegaKI - ComplexDivArg(omegaKI, pK_re, pK_im, pI_re, pI_im);

                double nBranch = Math.Round((alphaL - alphaI) / (2.0 * Math.PI));
                alphaJ -= 2.0 * Math.PI * nBranch / 3.0;
                alphaK -= 4.0 * Math.PI * nBranch / 3.0;

                if (Math.Abs(nBranch) >= 1.0)
                {
                    branchPoints.Add(mesh.FaceCenters[f]);
                    continue;
                }

                // Extract all iso-lines alpha = 2*pi*m inside triangle (vi, vj, vk)
                double minA = Math.Min(alphaI, Math.Min(alphaJ, alphaK));
                double maxA = Math.Max(alphaI, Math.Max(alphaJ, alphaK));
                int mStart = (int)Math.Ceiling(minA / (2.0 * Math.PI));
                int mEnd = (int)Math.Floor(maxA / (2.0 * Math.PI));

                Point3d pI = mesh.Positions[vi];
                Point3d pJ = mesh.Positions[vj];
                Point3d pK = mesh.Positions[vk];

                for (int m = mStart; m <= mEnd; m++)
                {
                    double target = 2.0 * Math.PI * m;
                    var crossPts = new List<Point3d>(2);
                    TryTriangleEdgeIso(pI, pJ, alphaI - target, alphaJ - target, crossPts);
                    TryTriangleEdgeIso(pJ, pK, alphaJ - target, alphaK - target, crossPts);
                    TryTriangleEdgeIso(pK, pI, alphaK - target, alphaI - target, crossPts);

                    if (crossPts.Count >= 2 && crossPts[0].DistanceToSquared(crossPts[1]) > 1e-14)
                    {
                        stripeSegments.Add(new Line(crossPts[0], crossPts[1]));
                    }
                }
            }

            equidistantStripeCurves = new List<Polyline>();
            if (stripeSegments.Count > 0)
            {
                Curve[] joined = Curve.JoinCurves(stripeSegments.ConvertAll(s => (Curve)new LineCurve(s)), mesh.MeanEdgeLength * 0.05);
                if (joined != null)
                {
                    foreach (var crv in joined)
                    {
                        if (crv.TryGetPolyline(out Polyline pl))
                            equidistantStripeCurves.Add(pl);
                    }
                }
            }

            return stripeScalar;
        }

        private static double ComplexDivArg(double omega, double reA, double imA, double reB, double imB)
        {
            // Computes arg( exp(i*omega) * A / B ) = arg( exp(i*omega) * A * conj(B) )
            double cosO = Math.Cos(omega), sinO = Math.Sin(omega);
            // rA = exp(i*omega) * A
            double rA_re = cosO * reA - sinO * imA;
            double rA_im = sinO * reA + cosO * imA;
            // prod = rA * conj(B)
            double prod_re = rA_re * reB + rA_im * imB;
            double prod_im = rA_im * reB - rA_re * imB;
            return Math.Atan2(prod_im, prod_re);
        }

        private static void TryTriangleEdgeIso(Point3d pA, Point3d pB, double vA, double vB, List<Point3d> pts)
        {
            if ((vA <= 0.0 && vB >= 0.0) || (vB <= 0.0 && vA >= 0.0))
            {
                double denom = vA - vB;
                if (Math.Abs(denom) > 1e-14)
                {
                    double t = Math.Max(0.0, Math.Min(1.0, vA / denom));
                    pts.Add(pA + t * (pB - pA));
                }
            }
        }

        // =========================================================================================
        // 6. SPIN TRANSFORMATIONS OF DISCRETE SURFACES (Crane, Pinkall, Schröder 2011)
        // =========================================================================================
        public static Mesh ComputeSpinTransformation(
            DdgMesh mesh,
            IList<double> rhoInput,
            out double[] spinNorms)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;

            // Map rhoInput whether user passed per-face, per-vertex, or a single scalar!
            double[] rhoPerFace = new double[nF];
            if (rhoInput != null && rhoInput.Count > 0)
            {
                if (rhoInput.Count == nF)
                {
                    for (int f = 0; f < nF; f++) rhoPerFace[f] = rhoInput[f];
                }
                else if (rhoInput.Count == nV || rhoInput.Count == mesh.RawToDdgVertex.Length)
                {
                    double[] vRho = mesh.MapScalarInputToVertices(rhoInput);
                    for (int f = 0; f < nF; f++)
                        rhoPerFace[f] = (vRho[mesh.Faces[f][0]] + vRho[mesh.Faces[f][1]] + vRho[mesh.Faces[f][2]]) / 3.0;
                }
                else if (rhoInput.Count == 1)
                {
                    // Modulate by mean curvature variation so a single slider value produces visible conformal deformation!
                    double scaleVal = rhoInput[0];
                    for (int f = 0; f < nF; f++)
                    {
                        double hAvg = (mesh.MeanCurvature[mesh.Faces[f][0]] +
                                       mesh.MeanCurvature[mesh.Faces[f][1]] +
                                       mesh.MeanCurvature[mesh.Faces[f][2]]) / 3.0;
                        rhoPerFace[f] = scaleVal * hAvg;
                    }
                }
                else
                {
                    for (int f = 0; f < nF; f++)
                        rhoPerFace[f] = rhoInput[f % rhoInput.Count];
                }
            }

            Point3d centroid = Point3d.Origin;
            for (int i = 0; i < nV; i++) centroid += mesh.Positions[i];
            centroid /= nV;

            double maxRadius = 1e-8;
            Point3d[] normPos = new Point3d[nV];
            for (int i = 0; i < nV; i++)
            {
                Vector3d d = mesh.Positions[i] - centroid;
                if (d.Length > maxRadius) maxRadius = d.Length;
                normPos[i] = new Point3d(d.X, d.Y, d.Z);
            }
            for (int i = 0; i < nV; i++)
                normPos[i] /= maxRadius;

            var diracBuilder = new SparseMatrixBuilder(4 * nV, 4 * nV);
            for (int f = 0; f < nF; f++)
            {
                int i0 = mesh.Faces[f][0], i1 = mesh.Faces[f][1], i2 = mesh.Faces[f][2];
                int[] I = new int[] { i0, i1, i2 };

                Vector3d p0 = new Vector3d(normPos[i0]);
                Vector3d p1 = new Vector3d(normPos[i1]);
                Vector3d p2 = new Vector3d(normPos[i2]);

                double area = Math.Max(1e-12, 0.5 * Vector3d.CrossProduct(p1 - p0, p2 - p0).Length);
                double rho = rhoPerFace[f];

                double a = -1.0 / (4.0 * area);
                double b = rho / 6.0;
                double c = area * rho * rho / 9.0;

                Vector3d[] e = new Vector3d[3];
                for (int k = 0; k < 3; k++)
                    e[k] = new Vector3d(normPos[I[(k + 2) % 3]] - normPos[I[(k + 1) % 3]]);

                for (int m = 0; m < 3; m++)
                {
                    for (int n = 0; n < 3; n++)
                    {
                        double qw = a * (-(e[m] * e[n])) + c;
                        Vector3d qv = a * Vector3d.CrossProduct(e[m], e[n]) + b * (e[n] - e[m]);
                        AddQuaternionBlock(diracBuilder, I[m], I[n], qw, qv.X, qv.Y, qv.Z);
                    }
                }
            }

            diracBuilder.ShiftDiagonal(1e-4);
            CsrMatrix diracMat = diracBuilder.Build();
            double[] spinVec = SparseLinearSolver.SmallestEigenvector(diracMat, null, 10, 400);

            spinNorms = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                double w = spinVec[4 * i], x = spinVec[4 * i + 1], y = spinVec[4 * i + 2], z = spinVec[4 * i + 3];
                spinNorms[i] = Math.Sqrt(w * w + x * x + y * y + z * z);
            }

            double[] omegaX = new double[nV];
            double[] omegaY = new double[nV];
            double[] omegaZ = new double[nV];

            for (int f = 0; f < nF; f++)
            {
                int[] v = mesh.Faces[f];
                for (int j = 0; j < 3; j++)
                {
                    int k0 = v[j];
                    int a = v[(j + 1) % 3];
                    int b = v[(j + 2) % 3];
                    if (a > b) { int tmp = a; a = b; b = tmp; }

                    Vector3d edgeVec = new Vector3d(normPos[b] - normPos[a]);
                    double[] l1 = GetQuat(spinVec, a);
                    double[] l2 = GetQuat(spinVec, b);

                    Vector3d eTilde = (1.0 / 3.0) * SandwichQuat(l1, edgeVec, l1) +
                                      (1.0 / 6.0) * SandwichQuat(l1, edgeVec, l2) +
                                      (1.0 / 6.0) * SandwichQuat(l2, edgeVec, l1) +
                                      (1.0 / 3.0) * SandwichQuat(l2, edgeVec, l2);

                    Vector3d u1 = new Vector3d(normPos[v[(j + 1) % 3]] - normPos[k0]);
                    Vector3d u2 = new Vector3d(normPos[v[(j + 2) % 3]] - normPos[k0]);
                    double sinA = Math.Max(1e-10, Vector3d.CrossProduct(u1, u2).Length);
                    double cotAlpha = Math.Max(-50.0, Math.Min(50.0, (u1 * u2) / sinA));

                    Vector3d contrib = 0.5 * cotAlpha * eTilde;
                    omegaX[a] -= contrib.X; omegaY[a] -= contrib.Y; omegaZ[a] -= contrib.Z;
                    omegaX[b] += contrib.X; omegaY[b] += contrib.Y; omegaZ[b] += contrib.Z;
                }
            }

            var lapBuilder = mesh.BuildCotanLaplacianBuilder(1.0);
            lapBuilder.PinRow(nV - 1, 1.0);
            omegaX[nV - 1] = 0.0; omegaY[nV - 1] = 0.0; omegaZ[nV - 1] = 0.0;
            CsrMatrix lapMat = lapBuilder.Build();

            double[] nx = SparseLinearSolver.SolvePCG(lapMat, omegaX, 1200, 1e-7);
            double[] ny = SparseLinearSolver.SolvePCG(lapMat, omegaY, 1200, 1e-7);
            double[] nz = SparseLinearSolver.SolvePCG(lapMat, omegaZ, 1200, 1e-7);

            Point3d newCenter = Point3d.Origin;
            for (int i = 0; i < nV; i++)
                newCenter += new Point3d(nx[i], ny[i], nz[i]);
            newCenter /= nV;

            double newMaxR = 1e-8;
            for (int i = 0; i < nV; i++)
            {
                double d = new Point3d(nx[i], ny[i], nz[i]).DistanceTo(newCenter);
                if (d > newMaxR) newMaxR = d;
            }

            Mesh outMesh = mesh.RhinoMesh.DuplicateMesh();
            double scaleRestore = maxRadius / newMaxR;
            for (int i = 0; i < nV; i++)
            {
                Point3d p = centroid + (new Point3d(nx[i], ny[i], nz[i]) - newCenter) * scaleRestore;
                outMesh.Vertices.SetVertex(i, p);
            }
            outMesh.Normals.ComputeNormals();
            return outMesh;
        }

        // =========================================================================================
        // 7. BOUNDARY FIRST FLATTENING / CONFORMAL FLATTENING (Sawhney & Crane 2017)
        // =========================================================================================
        public static Mesh ComputeConformalFlattening(
            DdgMesh mesh,
            int boundaryMode,
            out double[] scaleFactors)
        {
            int nV = mesh.VertexCount;
            List<int> loop = ExtractOrderedBoundaryLoop(mesh);

            // If mesh is closed (no boundary), puncture triangle 0 so its 3 vertices form the outer boundary!
            if (loop.Count < 3 && mesh.FaceCount > 0)
            {
                loop.Clear();
                loop.Add(mesh.Faces[0][0]);
                loop.Add(mesh.Faces[0][1]);
                loop.Add(mesh.Faces[0][2]);
            }

            var loopSet = new HashSet<int>(loop);
            var yamabeBuilder = mesh.BuildCotanLaplacianBuilder(1.0);
            double[] rhsScale = new double[nV];

            for (int i = 0; i < nV; i++)
            {
                if (loopSet.Contains(i))
                {
                    if (boundaryMode == 0)
                    {
                        yamabeBuilder.PinRow(i, 1.0);
                        rhsScale[i] = 0.0;
                    }
                    else
                    {
                        rhsScale[i] = mesh.GaussianCurvature[i] - (2.0 * Math.PI / Math.Max(3, loop.Count));
                    }
                }
                else
                {
                    rhsScale[i] = mesh.GaussianCurvature[i];
                }
            }
            yamabeBuilder.ShiftDiagonal(1e-7);
            scaleFactors = SparseLinearSolver.SolvePCG(yamabeBuilder.Build(), rhsScale, 1200, 1e-8);

            var lapU = mesh.BuildCotanLaplacianBuilder(1.0);
            var lapV = mesh.BuildCotanLaplacianBuilder(1.0);
            double[] rhsU = new double[nV];
            double[] rhsV = new double[nV];

            if (loop.Count >= 3)
            {
                if (boundaryMode == 1 || loop.Count == 3)
                {
                    double totalLen = 0.0;
                    double[] cumLen = new double[loop.Count];
                    for (int k = 0; k < loop.Count; k++)
                    {
                        int vA = loop[k];
                        int vB = loop[(k + 1) % loop.Count];
                        cumLen[k] = totalLen;
                        totalLen += mesh.Positions[vA].DistanceTo(mesh.Positions[vB]);
                    }
                    double radius = totalLen / (2.0 * Math.PI);
                    for (int k = 0; k < loop.Count; k++)
                    {
                        int idx = loop[k];
                        double ang = 2.0 * Math.PI * cumLen[k] / Math.Max(1e-8, totalLen);
                        lapU.PinRow(idx, 1.0);
                        lapV.PinRow(idx, 1.0);
                        rhsU[idx] = radius * Math.Cos(ang);
                        rhsV[idx] = radius * Math.Sin(ang);
                    }
                }
                else
                {
                    Point2d curr = Point2d.Origin;
                    double tangentAngle = 0.0;
                    Point2d[] bndPos = new Point2d[loop.Count];
                    double totalAngleDefect = 0.0;
                    for (int k = 0; k < loop.Count; k++)
                        totalAngleDefect += mesh.GaussianCurvature[loop[k]];
                    double scaleAngle = Math.Abs(totalAngleDefect) > 1e-6 ? (2.0 * Math.PI / totalAngleDefect) : 1.0;

                    for (int k = 0; k < loop.Count; k++)
                    {
                        bndPos[k] = curr;
                        int vA = loop[k];
                        int vB = loop[(k + 1) % loop.Count];
                        double factor = Math.Max(-3.0, Math.Min(3.0, 0.5 * (scaleFactors[vA] + scaleFactors[vB])));
                        double len = mesh.Positions[vA].DistanceTo(mesh.Positions[vB]) * Math.Exp(factor);
                        curr += new Vector2d(len * Math.Cos(tangentAngle), len * Math.Sin(tangentAngle));
                        tangentAngle += mesh.GaussianCurvature[vB] * scaleAngle;
                    }

                    Vector2d closureError = curr - Point2d.Origin;
                    for (int k = 0; k < loop.Count; k++)
                    {
                        double frac = (double)k / loop.Count;
                        Point2d p = bndPos[k] - frac * closureError;
                        int idx = loop[k];
                        lapU.PinRow(idx, 1.0);
                        lapV.PinRow(idx, 1.0);
                        rhsU[idx] = p.X;
                        rhsV[idx] = p.Y;
                    }
                }
            }

            double[] uCoord = SparseLinearSolver.SolvePCG(lapU.Build(), rhsU, 1500, 1e-8);
            double[] vCoord = SparseLinearSolver.SolvePCG(lapV.Build(), rhsV, 1500, 1e-8);

            Mesh flatMesh = mesh.RhinoMesh.DuplicateMesh();
            flatMesh.TextureCoordinates.Clear();
            for (int i = 0; i < nV; i++)
            {
                flatMesh.Vertices.SetVertex(i, uCoord[i], vCoord[i], 0.0);
                flatMesh.TextureCoordinates.Add(uCoord[i], vCoord[i]);
            }
            flatMesh.Normals.ComputeNormals();
            return flatMesh;
        }

        // =========================================================================================
        // 8. FEATURE-AWARE HEATMAP & MESH SEGMENTATION (Position + Normal + Sharp Crease Barriers)
        // =========================================================================================
        public static double[] ComputeFeatureAwareHeatmap(
            DdgMesh mesh,
            IEnumerable<int> sourceVertices,
            double barrierStrength,
            double diffusionTimeFactor)
        {
            int n = mesh.VertexCount;
            double h = mesh.MeanEdgeLength;
            double t = Math.Max(0.1, diffusionTimeFactor) * 4.0 * h * h;

            // Build anisotropic feature-barrier Laplacian where sharp edges block heat diffusion
            var builder = mesh.BuildFeatureLaplacianBuilder(t, barrierStrength);
            for (int i = 0; i < n; i++)
                builder.Add(i, i, mesh.DualAreas[i]);

            const double pinWeight = 1e6;
            double[] rhs = new double[n];
            bool anySrc = false;
            foreach (int sv in sourceVertices)
            {
                if (sv >= 0 && sv < n)
                {
                    builder.Add(sv, sv, pinWeight);
                    rhs[sv] = pinWeight * 1.0;
                    anySrc = true;
                }
            }
            if (!anySrc && n > 0)
            {
                builder.Add(0, 0, pinWeight);
                rhs[0] = pinWeight * 1.0;
            }

            double[] heat = SparseLinearSolver.SolvePCG(builder.Build(), rhs, 1200, 1e-8);

            double minH = double.MaxValue, maxH = double.MinValue;
            for (int i = 0; i < n; i++)
            {
                if (heat[i] < minH) minH = heat[i];
                if (heat[i] > maxH) maxH = heat[i];
            }
            double span = Math.Max(1e-12, maxH - minH);
            for (int i = 0; i < n; i++)
                heat[i] = (heat[i] - minH) / span;

            return heat;
        }

        /// <summary>
        /// Segments a mesh into K distinct pieces based on vertex/face Position proximity,
        /// Normal similarity, Sharp Dihedral Crease Edges, and optional Vertex Weights (e.g. Gaussian Curvature folds).
        /// </summary>
        public static List<Mesh> ComputeMeshHeatmapSegmentation(
            DdgMesh mesh,
            int numSegments,
            IList<Point3d> customSeedPoints,
            double creaseAndNormalWeight,
            IList<double> optionalVertexWeights,
            out List<Polyline> boundaryCurves,
            out int[] faceLabels,
            out Mesh coloredSegmentedMesh)
        {
            int nF = mesh.FaceCount;
            int K = Math.Max(2, Math.Min(nF, numSegments));

            // Map optional vertex weights (e.g. Gaussian curvature K or custom fold weights) to [0, 1]
            double[] normVertWeights = new double[mesh.VertexCount];
            if (optionalVertexWeights != null && optionalVertexWeights.Count > 0)
            {
                double[] rawW = mesh.MapScalarInputToVertices(optionalVertexWeights);
                double maxAbs = 1e-12;
                for (int i = 0; i < rawW.Length; i++)
                    if (Math.Abs(rawW[i]) > maxAbs) maxAbs = Math.Abs(rawW[i]);
                for (int i = 0; i < rawW.Length; i++)
                    normVertWeights[i] = Math.Abs(rawW[i]) / maxAbs;
            }
            else
            {
                // Default subtle Gaussian curvature fold boost
                double maxK = 1e-12;
                for (int i = 0; i < mesh.VertexCount; i++)
                    if (Math.Abs(mesh.GaussianCurvature[i]) > maxK) maxK = Math.Abs(mesh.GaussianCurvature[i]);
                for (int i = 0; i < mesh.VertexCount; i++)
                    normVertWeights[i] = 0.35 * Math.Abs(mesh.GaussianCurvature[i]) / maxK;
            }

            // 1. Build dual graph adjacency and edge crossing cost between adjacent triangles
            var adjFaces = new List<(int neighborFace, double cost)>[nF];
            for (int f = 0; f < nF; f++)
                adjFaces[f] = new List<(int, double)>(3);

            double beta = Math.Max(0.0, creaseAndNormalWeight);
            double meanDist = mesh.MeanEdgeLength;

            foreach (var e in mesh.Edges)
            {
                if (e.HalfEdge1 == -1) continue;
                int f0 = mesh.HalfEdges[e.HalfEdge0].FaceIndex;
                int f1 = mesh.HalfEdges[e.HalfEdge1].FaceIndex;

                double posDist = mesh.FaceCenters[f0].DistanceTo(mesh.FaceCenters[f1]);
                double normDiff = 1.0 - Math.Max(-1.0, Math.Min(1.0, mesh.FaceNormals[f0] * mesh.FaceNormals[f1]));
                double creasePenalty = e.DihedralAngle * e.DihedralAngle;
                if (e.DihedralAngle > 0.42) creasePenalty *= 8.0;
                if (e.SignedDihedralAngle < -0.1) creasePenalty *= 2.0;

                // Vertex weight / Gaussian curvature fold barrier along edge (v0, v1)
                double foldWeight = 0.5 * (normVertWeights[e.V0] + normVertWeights[e.V1]);

                double cost = posDist + meanDist * beta * (2.5 * normDiff + 4.0 * creasePenalty + 5.0 * foldWeight);
                adjFaces[f0].Add((f1, cost));
                adjFaces[f1].Add((f0, cost));
            }

            // 2. Initialize seed faces (from customSeedPoints if provided, else Farthest-Point Feature Geodesic sampling)
            var seedFaces = new List<int>();
            if (customSeedPoints != null && customSeedPoints.Count >= 2)
            {
                foreach (var pt in customSeedPoints)
                {
                    MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(pt, 0.0);
                    if (mp != null && !seedFaces.Contains(mp.FaceIndex))
                        seedFaces.Add(mp.FaceIndex);
                }
                K = seedFaces.Count;
            }
            else
            {
                seedFaces.Add(0);
                double[] minDistToSeeds = new double[nF];
                for (int f = 0; f < nF; f++) minDistToSeeds[f] = double.MaxValue;

                for (int s = 0; s < K; s++)
                {
                    int latestSeed = seedFaces[seedFaces.Count - 1];
                    double[] dFromLatest = RunFaceDijkstra(nF, adjFaces, new int[] { latestSeed }, out _);
                    int farthestFace = 0;
                    double maxD = -1.0;
                    for (int f = 0; f < nF; f++)
                    {
                        if (dFromLatest[f] < minDistToSeeds[f])
                            minDistToSeeds[f] = dFromLatest[f];
                        if (minDistToSeeds[f] > maxD)
                        {
                            maxD = minDistToSeeds[f];
                            farthestFace = f;
                        }
                    }
                    if (s < K - 1)
                        seedFaces.Add(farthestFace);
                }
            }

            // 3. Perform 3 iterations of Geodesic-Normal Centroidal Voronoi relaxation so segments are balanced and lock to sharp edges
            faceLabels = new int[nF];
            for (int iter = 0; iter < 3; iter++)
            {
                RunFaceDijkstra(nF, adjFaces, seedFaces, out faceLabels);

                if (customSeedPoints != null && customSeedPoints.Count >= 2)
                    break; // Keep user seeds fixed

                // Update each cluster's medoid face (closest to cluster average position + normal)
                var clusterPos = new Vector3d[K];
                var clusterNorm = new Vector3d[K];
                var clusterArea = new double[K];

                for (int f = 0; f < nF; f++)
                {
                    int cid = faceLabels[f];
                    double a = mesh.FaceAreas[f];
                    clusterPos[cid] += new Vector3d(mesh.FaceCenters[f]) * a;
                    clusterNorm[cid] += mesh.FaceNormals[f] * a;
                    clusterArea[cid] += a;
                }

                for (int c = 0; c < K; c++)
                {
                    if (clusterArea[c] < 1e-12) continue;
                    Point3d avgP = new Point3d(clusterPos[c] / clusterArea[c]);
                    Vector3d avgN = clusterNorm[c];
                    if (avgN.Length > 1e-12) avgN.Unitize();

                    int bestF = seedFaces[c];
                    double bestScore = double.MaxValue;
                    for (int f = 0; f < nF; f++)
                    {
                        if (faceLabels[f] != c) continue;
                        double score = mesh.FaceCenters[f].DistanceTo(avgP) +
                                       meanDist * beta * (1.0 - (mesh.FaceNormals[f] * avgN));
                        if (score < bestScore)
                        {
                            bestScore = score;
                            bestF = f;
                        }
                    }
                    seedFaces[c] = bestF;
                }
            }

            // 4. Build separate Sub-Meshes, Boundary Cut Curves, and Colored Preview Mesh
            Color[] palette = GenerateDistinctPalette(K);
            coloredSegmentedMesh = new Mesh();
            var subMeshes = new List<Mesh>(K);

            for (int c = 0; c < K; c++)
            {
                var sub = new Mesh();
                var oldToNewV = new Dictionary<int, int>();

                for (int f = 0; f < nF; f++)
                {
                    if (faceLabels[f] != c) continue;
                    int[] fv = mesh.Faces[f];
                    int[] nv = new int[3];
                    for (int k = 0; k < 3; k++)
                    {
                        int origV = fv[k];
                        if (!oldToNewV.TryGetValue(origV, out int newIdx))
                        {
                            newIdx = sub.Vertices.Add(mesh.Positions[origV]);
                            sub.VertexColors.Add(palette[c]);
                            oldToNewV[origV] = newIdx;
                        }
                        nv[k] = newIdx;
                    }
                    sub.Faces.AddFace(nv[0], nv[1], nv[2]);
                }

                if (sub.Faces.Count > 0)
                {
                    sub.Normals.ComputeNormals();
                    subMeshes.Add(sub);
                    coloredSegmentedMesh.Append(sub);
                }
            }

            // Extract segmentation boundary edges (where adjacent faces have different segment labels)
            var cutLines = new List<Line>();
            foreach (var e in mesh.Edges)
            {
                if (e.HalfEdge1 == -1) continue;
                int f0 = mesh.HalfEdges[e.HalfEdge0].FaceIndex;
                int f1 = mesh.HalfEdges[e.HalfEdge1].FaceIndex;
                if (faceLabels[f0] != faceLabels[f1])
                {
                    cutLines.Add(new Line(mesh.Positions[e.V0], mesh.Positions[e.V1]));
                }
            }

            boundaryCurves = new List<Polyline>();
            if (cutLines.Count > 0)
            {
                Curve[] joined = Curve.JoinCurves(cutLines.ConvertAll(l => (Curve)new LineCurve(l)), meanDist * 1e-3);
                if (joined != null)
                {
                    foreach (var crv in joined)
                        if (crv.TryGetPolyline(out Polyline pl))
                            boundaryCurves.Add(pl);
                }
            }

            return subMeshes;
        }

        private static double[] RunFaceDijkstra(
            int nF,
            List<(int neighborFace, double cost)>[] adjFaces,
            IList<int> seeds,
            out int[] nearestSeedLabel)
        {
            double[] dist = new double[nF];
            nearestSeedLabel = new int[nF];
            for (int i = 0; i < nF; i++) dist[i] = double.MaxValue;

            var pq = new SortedSet<(double d, int f)>();
            for (int s = 0; s < seeds.Count; s++)
            {
                int sf = seeds[s];
                if (sf >= 0 && sf < nF)
                {
                    dist[sf] = 0.0;
                    nearestSeedLabel[sf] = s;
                    pq.Add((0.0, sf));
                }
            }

            while (pq.Count > 0)
            {
                var top = pq.Min;
                pq.Remove(top);
                int u = top.f;
                if (top.d > dist[u] + 1e-12) continue;

                foreach (var edge in adjFaces[u])
                {
                    int v = edge.neighborFace;
                    double nd = dist[u] + edge.cost;
                    if (nd + 1e-12 < dist[v])
                    {
                        pq.Remove((dist[v], v));
                        dist[v] = nd;
                        nearestSeedLabel[v] = nearestSeedLabel[u];
                        pq.Add((nd, v));
                    }
                }
            }
            return dist;
        }

        private static Color[] GenerateDistinctPalette(int count)
        {
            // Keenan Crane / Polyscope inspired high-contrast scientific pastel/vibrant palette
            Color[] baseColors = new Color[]
            {
                Color.FromArgb(230, 75, 53),   // Crimson Red
                Color.FromArgb(77, 187, 213),  // Cyan Blue
                Color.FromArgb(0, 160, 135),   // Teal Green
                Color.FromArgb(243, 155, 127), // Warm Peach/Gold
                Color.FromArgb(60, 84, 136),   // Royal Indigo
                Color.FromArgb(132, 145, 180), // Slate Lavender
                Color.FromArgb(220, 180, 60),  // Amber Yellow
                Color.FromArgb(170, 90, 160)   // Magenta Plum
            };

            Color[] res = new Color[count];
            for (int i = 0; i < count; i++)
            {
                if (i < baseColors.Length)
                    res[i] = baseColors[i];
                else
                {
                    double hue = (i * 0.61803398875) % 1.0;
                    res[i] = HsvToRgb(hue * 360.0, 0.72, 0.90);
                }
            }
            return res;
        }

        private static Color HsvToRgb(double h, double s, double v)
        {
            int hi = (int)Math.Floor(h / 60.0) % 6;
            double f = h / 60.0 - Math.Floor(h / 60.0);
            int p = (int)(255 * v * (1 - s));
            int q = (int)(255 * v * (1 - f * s));
            int t = (int)(255 * v * (1 - (1 - f) * s));
            int val = (int)(255 * v);
            switch (hi)
            {
                case 0: return Color.FromArgb(val, t, p);
                case 1: return Color.FromArgb(q, val, p);
                case 2: return Color.FromArgb(p, val, t);
                case 3: return Color.FromArgb(p, q, val);
                case 4: return Color.FromArgb(t, p, val);
                default: return Color.FromArgb(val, p, q);
            }
        }

        // =========================================================================================
        // 9. GEODESIC CURVE NETWORK SHORTENING (FlipOut Curve Networks: Sharp & Crane 2020)
        //    Supports single curves, closed loops, multi-curve networks with moving triple junctions,
        //    T-junction endpoints that slide freely along other curves, AND optional Vertex Weight
        //    attraction (W) so curves snap into Gaussian curvature folds / creases!
        // =========================================================================================
        public static List<Polyline> ComputeGeodesicNetworkShorten(
            Mesh mesh,
            IList<Curve> inputCurves,
            int iterations,
            bool allowFreeEndpointsToSlideOnOtherCurves,
            out double totalLength,
            IList<double> optionalVertexWeights = null)
        {
            totalLength = 0.0;
            var result = new List<Polyline>();
            if (mesh == null || inputCurves == null || inputCurves.Count == 0) return result;

            DdgMesh ddg = null;
            Vector3d[] weightGrad = null;
            if (optionalVertexWeights != null && optionalVertexWeights.Count > 0)
            {
                ddg = new DdgMesh(mesh);
                double[] w = ddg.MapScalarInputToVertices(optionalVertexWeights);
                double maxW = 1e-12;
                for (int i = 0; i < w.Length; i++)
                    if (Math.Abs(w[i]) > maxW) maxW = Math.Abs(w[i]);
                for (int i = 0; i < w.Length; i++)
                    w[i] = Math.Abs(w[i]) / maxW;

                ddg.ComputeGradient(w, out _, out weightGrad);
                for (int i = 0; i < weightGrad.Length; i++)
                {
                    if (weightGrad[i].Length > 1e-12)
                        weightGrad[i].Unitize();
                }
            }

            BoundingBox bbox = mesh.GetBoundingBox(false);
            double snapTol = Math.Max(1e-4, bbox.Diagonal.Length * 0.015);

            // 1. Discretize all input curves into polylines snapped onto the mesh
            int numCrvs = inputCurves.Count;
            var polyPts = new List<Point3d[]>();
            var isClosed = new List<bool>();

            for (int c = 0; c < numCrvs; c++)
            {
                Curve crv = inputCurves[c];
                if (crv == null) continue;

                Point3d[] pts;
                if (crv.TryGetPolyline(out Polyline existingPl) && existingPl.Count >= 4 && existingPl.Count <= 120)
                {
                    int cnt = existingPl.IsClosed ? existingPl.Count - 1 : existingPl.Count;
                    pts = new Point3d[cnt];
                    for (int i = 0; i < cnt; i++) pts[i] = existingPl[i];
                    isClosed.Add(existingPl.IsClosed);
                }
                else
                {
                    crv.DivideByCount(40, true, out pts);
                    if (pts == null || pts.Length < 2) continue;
                    isClosed.Add(crv.IsClosed);
                }

                for (int i = 0; i < pts.Length; i++)
                {
                    MeshPoint mp = mesh.ClosestMeshPoint(pts[i], 0.0);
                    if (mp != null) pts[i] = mp.Point;
                }
                polyPts.Add(pts);
            }

            int mCount = polyPts.Count;
            if (mCount == 0) return result;

            // 2. Detect shared network junctions (where 2+ curve endpoints meet)
            //    and T-junction sliding endpoints (where a curve endpoint lies on the interior of another curve)
            var junctionGroups = new List<List<(int crvIdx, int ptIdx)>>();
            var endpointAssigned = new HashSet<(int, int)>();

            var allEndpoints = new List<(int crvIdx, int ptIdx, Point3d pos)>();
            for (int c = 0; c < mCount; c++)
            {
                if (isClosed[c]) continue;
                allEndpoints.Add((c, 0, polyPts[c][0]));
                allEndpoints.Add((c, polyPts[c].Length - 1, polyPts[c][polyPts[c].Length - 1]));
            }

            for (int i = 0; i < allEndpoints.Count; i++)
            {
                var epA = allEndpoints[i];
                if (endpointAssigned.Contains((epA.crvIdx, epA.ptIdx))) continue;

                var group = new List<(int crvIdx, int ptIdx)> { (epA.crvIdx, epA.ptIdx) };
                endpointAssigned.Add((epA.crvIdx, epA.ptIdx));

                for (int j = i + 1; j < allEndpoints.Count; j++)
                {
                    var epB = allEndpoints[j];
                    if (endpointAssigned.Contains((epB.crvIdx, epB.ptIdx))) continue;
                    if (epA.pos.DistanceTo(epB.pos) <= snapTol)
                    {
                        group.Add((epB.crvIdx, epB.ptIdx));
                        endpointAssigned.Add((epB.crvIdx, epB.ptIdx));
                    }
                }
                junctionGroups.Add(group);
            }

            var slidingOnCurve = new Dictionary<(int crvIdx, int ptIdx), int>();
            if (allowFreeEndpointsToSlideOnOtherCurves)
            {
                foreach (var group in junctionGroups)
                {
                    if (group.Count == 1)
                    {
                        var ep = group[0];
                        Point3d p = polyPts[ep.crvIdx][ep.ptIdx];
                        int bestRail = -1;
                        double bestDist = snapTol * 2.0;

                        for (int otherC = 0; otherC < mCount; otherC++)
                        {
                            if (otherC == ep.crvIdx) continue;
                            for (int k = 0; k < polyPts[otherC].Length; k++)
                            {
                                double d = p.DistanceTo(polyPts[otherC][k]);
                                if (d < bestDist)
                                {
                                    bestDist = d;
                                    bestRail = otherC;
                                }
                            }
                        }
                        if (bestRail != -1)
                            slidingOnCurve[ep] = bestRail;
                    }
                }
            }

            // 3. Iterative Geodesic Network Tensioning (+ optional Fold Weight Attraction) on the Mesh
            for (int step = 0; step < iterations; step++)
            {
                for (int c = 0; c < mCount; c++)
                {
                    Point3d[] pts = polyPts[c];
                    int n = pts.Length;
                    Point3d[] nextPts = new Point3d[n];
                    bool closed = isClosed[c];

                    for (int i = 0; i < n; i++)
                    {
                        if (!closed && (i == 0 || i == n - 1))
                        {
                            nextPts[i] = pts[i];
                            continue;
                        }
                        int prev = (i - 1 + n) % n;
                        int next = (i + 1) % n;
                        Point3d target = 0.5 * (pts[prev] + pts[next]);
                        Point3d candidate = 0.4 * pts[i] + 0.6 * target;

                        if (weightGrad != null && ddg != null)
                        {
                            int nearV = ddg.FindClosestVertex(pts[i]);
                            Vector3d tang = pts[next] - pts[prev];
                            if (tang.Length > 1e-12) tang.Unitize();
                            Vector3d gW = weightGrad[nearV];
                            // Transverse attraction toward high-weight fold ridge
                            Vector3d orthoGW = gW - (gW * tang) * tang;
                            candidate += 0.25 * ddg.MeanEdgeLength * orthoGW;
                        }

                        MeshPoint mp = mesh.ClosestMeshPoint(candidate, 0.0);
                        nextPts[i] = mp != null ? mp.Point : candidate;
                    }
                    polyPts[c] = nextPts;
                }

                foreach (var group in junctionGroups)
                {
                    if (group.Count >= 2)
                    {
                        Point3d avgPos = Point3d.Origin;
                        Vector3d tensionForce = Vector3d.Zero;
                        double minSegLen = double.MaxValue;

                        foreach (var (cIdx, pIdx) in group)
                        {
                            Point3d[] pts = polyPts[cIdx];
                            int neighborIdx = (pIdx == 0) ? 1 : pts.Length - 2;
                            avgPos += pts[pIdx];
                            Vector3d dir = pts[neighborIdx] - pts[pIdx];
                            double len = dir.Length;
                            if (len < minSegLen) minSegLen = len;
                            if (len > 1e-12)
                                tensionForce += dir / len;
                        }
                        avgPos /= group.Count;

                        Point3d movedJunction = avgPos + 0.25 * Math.Max(1e-4, minSegLen) * tensionForce;
                        MeshPoint mp = mesh.ClosestMeshPoint(movedJunction, 0.0);
                        Point3d snappedJunction = mp != null ? mp.Point : movedJunction;

                        foreach (var (cIdx, pIdx) in group)
                            polyPts[cIdx][pIdx] = snappedJunction;
                    }
                    else if (group.Count == 1 && slidingOnCurve.TryGetValue(group[0], out int railIdx))
                    {
                        var (cIdx, pIdx) = group[0];
                        Point3d[] pts = polyPts[cIdx];
                        int neighborIdx = (pIdx == 0) ? 1 : pts.Length - 2;
                        Point3d pullTarget = 0.5 * pts[pIdx] + 0.5 * pts[neighborIdx];

                        Point3d snappedOnRail = ProjectPointOntoPolyline(pullTarget, polyPts[railIdx], isClosed[railIdx]);
                        MeshPoint mp = mesh.ClosestMeshPoint(snappedOnRail, 0.0);
                        polyPts[cIdx][pIdx] = mp != null ? mp.Point : snappedOnRail;
                    }
                }
            }

            for (int c = 0; c < mCount; c++)
            {
                var pl = new Polyline(polyPts[c]);
                if (isClosed[c] && polyPts[c].Length > 0)
                    pl.Add(polyPts[c][0]);
                totalLength += pl.Length;
                result.Add(pl);
            }

            return result;
        }

        // =========================================================================================
        // 9B. VARIATIONAL SURFACE CUTTING (Sharp & Crane, SIGGRAPH 2018)
        //     Computes Yamabe distortion potential Lc * u = K + W and extracts optimal cut seams
        //     snapping to Gaussian curvature cones and user-weighted fold ridges (pliegues).
        // =========================================================================================
        public static List<Polyline> ComputeVariationalSurfaceCuts(
            DdgMesh mesh,
            IList<double> optionalVertexWeights,
            int numCutContours,
            out double[] distortionPotential,
            out Mesh coloredMesh)
        {
            int nV = mesh.VertexCount;

            // Combine intrinsic Gaussian curvature K with user vertex weights W (folds / creases)
            double[] w = new double[nV];
            if (optionalVertexWeights != null && optionalVertexWeights.Count > 0)
            {
                double[] mapped = mesh.MapScalarInputToVertices(optionalVertexWeights);
                double maxM = 1e-12;
                for (int i = 0; i < nV; i++)
                    if (Math.Abs(mapped[i]) > maxM) maxM = Math.Abs(mapped[i]);
                for (int i = 0; i < nV; i++)
                    w[i] = mapped[i] / maxM;
            }
            else
            {
                // Default: combine Gaussian curvature K and signed Mean Curvature folds
                double maxK = 1e-12;
                for (int i = 0; i < nV; i++)
                    if (Math.Abs(mesh.GaussianCurvature[i]) > maxK) maxK = Math.Abs(mesh.GaussianCurvature[i]);
                for (int i = 0; i < nV; i++)
                    w[i] = (mesh.GaussianCurvature[i] / maxK) + 0.5 * mesh.VertexSharpness[i];
            }

            // Solve regularized Yamabe / Phase-Field Poisson equation: (Lc + tau * M) u = K + W
            var builder = mesh.BuildCotanLaplacianBuilder(1.0);
            double tau = 0.05 / (mesh.MeanEdgeLength * mesh.MeanEdgeLength);
            double[] rhs = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                builder.Add(i, i, tau * mesh.DualAreas[i]);
                rhs[i] = mesh.GaussianCurvature[i] + w[i] * mesh.DualAreas[i];
            }

            distortionPotential = SparseLinearSolver.SolvePCG(builder.Build(), rhs, 1200, 1e-8);

            // Extract Variational Cut contours at key potential thresholds and tighten them toward high-weight folds
            double minU = double.MaxValue, maxU = double.MinValue;
            for (int i = 0; i < nV; i++)
            {
                if (distortionPotential[i] < minU) minU = distortionPotential[i];
                if (distortionPotential[i] > maxU) maxU = distortionPotential[i];
            }

            int nLevels = Math.Max(1, Math.Min(25, numCutContours));
            var levels = new List<double>(nLevels);
            for (int k = 1; k <= nLevels; k++)
            {
                double frac = (double)k / (nLevels + 1);
                levels.Add(minU + frac * (maxU - minU));
            }

            List<Polyline> rawContours = mesh.ExtractIsoContours(distortionPotential, levels);
            var rawCrvs = new List<Curve>(rawContours.Count);
            foreach (var pl in rawContours)
                rawCrvs.Add(pl.ToNurbsCurve());

            // Relax cut curves under the fold-weighted metric so they lock onto the folds (pliegues)
            List<Polyline> variationalCuts = ComputeGeodesicNetworkShorten(
                mesh.RhinoMesh, rawCrvs, 12, true, out _, w);

            coloredMesh = ColorMeshByScalar(mesh.RhinoMesh, distortionPotential, false);
            return variationalCuts;
        }

        private static Point3d ProjectPointOntoPolyline(Point3d pt, Point3d[] railPts, bool closed)
        {
            int n = railPts.Length;
            int segCount = closed ? n : n - 1;
            Point3d bestPt = railPts[0];
            double minDistSq = double.MaxValue;

            for (int i = 0; i < segCount; i++)
            {
                Point3d a = railPts[i];
                Point3d b = railPts[(i + 1) % n];
                Vector3d ab = b - a;
                double lenSq = ab.SquareLength;
                double t = lenSq > 1e-14 ? ((pt - a) * ab) / lenSq : 0.0;
                t = Math.Max(0.0, Math.Min(1.0, t));
                Point3d proj = a + t * ab;
                double dSq = pt.DistanceToSquared(proj);
                if (dSq < minDistSq)
                {
                    minDistSq = dSq;
                    bestPt = proj;
                }
            }
            return bestPt;
        }

        // =========================================================================================
        // 10. REPULSIVE CURVES: TANGENT-POINT ENERGY FLOW (Yu, Schumacher, Crane 2021)
        // =========================================================================================
        public static Polyline ComputeRepulsiveCurveFlow(
            Polyline inputCurve,
            double stepSize,
            int iterations,
            bool preserveEdgeLengths,
            out double finalEnergy)
        {
            bool isClosed = inputCurve.IsClosed;
            int n = isClosed ? inputCurve.Count - 1 : inputCurve.Count;
            Point3d[] pts = new Point3d[n];
            double[] restLengths = new double[n];

            for (int i = 0; i < n; i++)
                pts[i] = inputCurve[i];
            for (int i = 0; i < n; i++)
            {
                int next = (i + 1) % n;
                if (!isClosed && i == n - 1) break;
                restLengths[i] = pts[i].DistanceTo(pts[next]);
            }

            finalEnergy = 0.0;
            Vector3d[] forces = new Vector3d[n];

            for (int iter = 0; iter < iterations; iter++)
            {
                finalEnergy = 0.0;
                Array.Clear(forces, 0, n);

                for (int i = 0; i < n; i++)
                {
                    int prev = (i - 1 + n) % n;
                    int next = (i + 1) % n;
                    Vector3d tangent = !isClosed && i == 0 ? (pts[1] - pts[0])
                                     : !isClosed && i == n - 1 ? (pts[n - 1] - pts[n - 2])
                                     : (pts[next] - pts[prev]);
                    double ds = Math.Max(1e-8, 0.5 * tangent.Length);
                    if (tangent.Length > 1e-12) tangent.Unitize();

                    for (int j = 0; j < n; j++)
                    {
                        if (i == j || j == prev || j == next) continue;
                        Vector3d diff = pts[i] - pts[j];
                        double dist = diff.Length;
                        if (dist < 1e-8) continue;

                        double crossNorm = Vector3d.CrossProduct(tangent, diff).Length;
                        double energyTerm = (crossNorm * crossNorm) / Math.Pow(dist, 4.0) * ds;
                        finalEnergy += energyTerm;

                        Vector3d normalPart = diff - (diff * tangent) * tangent;
                        Vector3d repForce = (2.0 * normalPart / Math.Pow(dist, 4.0) + 2.0 * diff / Math.Pow(dist, 5.0)) * (ds * ds);
                        forces[i] += repForce;
                    }
                }

                double maxF = 1e-8;
                for (int i = 0; i < n; i++)
                    if (forces[i].Length > maxF) maxF = forces[i].Length;

                double scale = stepSize * restLengths[0] / maxF;
                for (int i = 0; i < n; i++)
                {
                    if (!isClosed && (i == 0 || i == n - 1)) continue;
                    pts[i] += forces[i] * scale;
                }

                if (preserveEdgeLengths)
                {
                    for (int relax = 0; relax < 3; relax++)
                    {
                        int segCount = isClosed ? n : n - 1;
                        for (int i = 0; i < segCount; i++)
                        {
                            int j = (i + 1) % n;
                            Vector3d d = pts[j] - pts[i];
                            double len = d.Length;
                            if (len > 1e-12)
                            {
                                Vector3d corr = 0.5 * (len - restLengths[i]) * (d / len);
                                if (isClosed || i > 0) pts[i] += corr;
                                if (isClosed || j < n - 1) pts[j] -= corr;
                            }
                        }
                    }
                }
            }

            var outPl = new Polyline(pts);
            if (isClosed && n > 0) outPl.Add(pts[0]);
            return outPl;
        }

        // =========================================================================================
        // 10. WALK ON SPHERES PDE SOLVER (Sawhney & Crane 2020)
        // =========================================================================================
        public static double[] ComputeWalkOnSpheres3D(
            Mesh boundaryMesh,
            IList<double> boundaryVertexValues,
            IList<Point3d> queryPoints,
            int numWalks = 64,
            int maxSteps = 16,
            double epsilon = 1e-2)
        {
            int nQ = queryPoints.Count;
            double[] results = new double[nQ];
            var rng = new Random(42);

            for (int q = 0; q < nQ; q++)
            {
                Point3d x0 = queryPoints[q];
                double sum = 0.0;

                for (int w = 0; w < numWalks; w++)
                {
                    Point3d x = x0;
                    MeshPoint mp = null;
                    for (int step = 0; step < maxSteps; step++)
                    {
                        mp = boundaryMesh.ClosestMeshPoint(x, 0.0);
                        if (mp == null) break;
                        double R = x.DistanceTo(mp.Point);
                        if (R <= epsilon) break;

                        double u = rng.NextDouble() * 2.0 - 1.0;
                        double phi = rng.NextDouble() * 2.0 * Math.PI;
                        double s = Math.Sqrt(Math.Max(0.0, 1.0 - u * u));
                        x += new Vector3d(R * s * Math.Cos(phi), R * s * Math.Sin(phi), R * u);
                    }

                    if (mp != null)
                    {
                        MeshFace f = boundaryMesh.Faces[mp.FaceIndex];
                        double vA = boundaryVertexValues[Math.Min(f.A, boundaryVertexValues.Count - 1)];
                        double vB = boundaryVertexValues[Math.Min(f.B, boundaryVertexValues.Count - 1)];
                        double vC = boundaryVertexValues[Math.Min(f.C, boundaryVertexValues.Count - 1)];
                        double val = mp.T[0] * vA + mp.T[1] * vB + mp.T[2] * vC;
                        if (f.IsQuad)
                        {
                            double vD = boundaryVertexValues[Math.Min(f.D, boundaryVertexValues.Count - 1)];
                            val += mp.T[3] * vD;
                        }
                        sum += val;
                    }
                }
                results[q] = numWalks > 0 ? sum / numWalks : 0.0;
            }
            return results;
        }

        // =========================================================================================
        // 11. SUBGRID MARCHING TETRAHEDRA (Baktash, Gillespie, Crane, ACM TOG 2026)
        // =========================================================================================
        public static Mesh ReconstructSubgridTetrahedron(Point3d[] tetCorners, int[] edgeCrossings)
        {
            int[,] edgePairs = new int[,] { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 2, 3 }, { 1, 3 }, { 1, 2 } };
            var mesh = new Mesh();
            int[][] edgePointIndices = new int[6][];

            for (int e = 0; e < 6; e++)
            {
                int count = Math.Max(0, edgeCrossings[e]);
                edgePointIndices[e] = new int[count];
                Point3d pA = tetCorners[edgePairs[e, 0]];
                Point3d pB = tetCorners[edgePairs[e, 1]];
                for (int k = 0; k < count; k++)
                {
                    double t = (k + 1.0) / (count + 1.0);
                    Point3d pt = pA + t * (pB - pA);
                    edgePointIndices[e][k] = mesh.Vertices.Add(pt);
                }
            }

            int maxCross = 0;
            for (int e = 0; e < 6; e++)
                if (edgeCrossings[e] > maxCross) maxCross = edgeCrossings[e];

            Point3d tetCenter = (tetCorners[0] + tetCorners[1] + tetCorners[2] + tetCorners[3]) * 0.25;

            for (int layer = 0; layer < maxCross; layer++)
            {
                var ringVerts = new List<int>();
                for (int e = 0; e < 6; e++)
                {
                    if (layer < edgePointIndices[e].Length)
                        ringVerts.Add(edgePointIndices[e][layer]);
                }

                if (ringVerts.Count == 3)
                {
                    mesh.Faces.AddFace(ringVerts[0], ringVerts[1], ringVerts[2]);
                }
                else if (ringVerts.Count >= 4)
                {
                    Point3d avg = Point3d.Origin;
                    foreach (int vi in ringVerts) avg += mesh.Vertices.Point3dAt(vi);
                    avg /= ringVerts.Count;

                    Vector3d n = Vector3d.CrossProduct(
                        mesh.Vertices.Point3dAt(ringVerts[1]) - avg,
                        mesh.Vertices.Point3dAt(ringVerts[2]) - avg);
                    if (n.Length < 1e-12) n = Vector3d.ZAxis;
                    n.Unitize();
                    Vector3d bx = mesh.Vertices.Point3dAt(ringVerts[0]) - avg;
                    bx -= (bx * n) * n;
                    bx.Unitize();
                    Vector3d by = Vector3d.CrossProduct(n, bx);

                    ringVerts.Sort((a, b) =>
                    {
                        Vector3d va = mesh.Vertices.Point3dAt(a) - avg;
                        Vector3d vb = mesh.Vertices.Point3dAt(b) - avg;
                        return Math.Atan2(va * by, va * bx).CompareTo(Math.Atan2(vb * by, vb * bx));
                    });

                    Point3d steiner = avg + 0.05 * (tetCenter - avg);
                    int steinerIdx = mesh.Vertices.Add(steiner);
                    for (int k = 0; k < ringVerts.Count; k++)
                    {
                        int v0 = ringVerts[k];
                        int v1 = ringVerts[(k + 1) % ringVerts.Count];
                        mesh.Faces.AddFace(v0, v1, steinerIdx);
                    }
                }
            }

            mesh.Normals.ComputeNormals();
            return mesh;
        }

        // =========================================================================================
        // 12. SCALAR & N-ROSY VECTOR FIELD SMOOTHING (Implicit Cotan & Connection Laplacian Diffusion)
        // =========================================================================================

        /// <summary>
        /// Smooths a noisy scalar field (such as Gaussian curvature K, Mean curvature H, or custom weights)
        /// via implicit diffusion (M + t * L) u_smooth = M * u_raw, with optional sharp-crease barrier B
        /// and outlier clamping before diffusion.
        /// </summary>
        public static double[] SmoothScalarField(
            DdgMesh mesh,
            IList<double> rawScalars,
            double smoothness,
            double creaseBarrierWeight,
            out Vector3d[] faceGradients,
            out Mesh coloredMesh)
        {
            int nV = mesh.VertexCount;
            double[] u0 = mesh.MapScalarInputToVertices(rawScalars);

            // Detect and clip only extreme outlier spikes while recording target domain [targetMin, targetMax]
            double[] sorted = (double[])u0.Clone();
            Array.Sort(sorted);
            double targetMin = sorted.Length > 0 ? sorted[0] : 0.0;
            double targetMax = sorted.Length > 0 ? sorted[sorted.Length - 1] : 1.0;

            if (nV >= 20)
            {
                double pLow = sorted[Math.Max(0, (int)(0.02 * nV))];
                double pHigh = sorted[Math.Min(nV - 1, (int)(0.98 * nV))];
                double innerSpan = pHigh - pLow;
                // Only clip if the extreme 2% tails are > 2.5x wider than the central 96% span (curvature singularity spikes)
                if (innerSpan > 1e-12 && (targetMax - targetMin) > 2.5 * innerSpan)
                {
                    targetMin = pLow;
                    targetMax = pHigh;
                    for (int i = 0; i < nV; i++)
                        u0[i] = Math.Max(pLow, Math.Min(pHigh, u0[i]));
                }
            }

            double diag = mesh.RhinoMesh.GetBoundingBox(false).Diagonal.Length;
            double hEff = Math.Max(2.2 * mesh.MeanEdgeLength, 0.05 * diag);
            double t = Math.Max(0.0, smoothness) * hEff * hEff;

            double[] uSmooth;
            if (t <= 1e-12)
            {
                uSmooth = u0;
            }
            else
            {
                SparseMatrixBuilder builder = creaseBarrierWeight > 1e-6
                    ? mesh.BuildFeatureLaplacianBuilder(creaseBarrierWeight, 2.0)
                    : mesh.BuildCotanLaplacianBuilder(1.0);

                // Scale Laplacian by diffusion time t and add Mass Matrix M on diagonal: (M + t*L) u = M * u0
                var sysBuilder = new SparseMatrixBuilder(nV, nV);
                CsrMatrix L = builder.Build();
                for (int i = 0; i < nV; i++)
                {
                    sysBuilder.Add(i, i, mesh.DualAreas[i]);
                    for (int k = L.RowPtr[i]; k < L.RowPtr[i + 1]; k++)
                    {
                        sysBuilder.Add(i, L.ColIdx[k], t * L.Values[k]);
                    }
                }

                double[] rhs = new double[nV];
                for (int i = 0; i < nV; i++)
                    rhs[i] = mesh.DualAreas[i] * u0[i];

                // Perform 2-step cascaded implicit biharmonic-like smoothing for ultra-clean level sets
                CsrMatrix sysMat = sysBuilder.Build();
                double[] mid = SparseLinearSolver.SolvePCG(sysMat, rhs, 1200, 1e-8);
                for (int i = 0; i < nV; i++)
                    rhs[i] = mesh.DualAreas[i] * mid[i];
                uSmooth = SparseLinearSolver.SolvePCG(sysMat, rhs, 1200, 1e-8);

                // Range-preserving affine rescaling back to [targetMin, targetMax] so Iso-Values never shrink toward the mean!
                double minS = double.MaxValue, maxS = double.MinValue;
                for (int i = 0; i < nV; i++)
                {
                    if (uSmooth[i] < minS) minS = uSmooth[i];
                    if (uSmooth[i] > maxS) maxS = uSmooth[i];
                }
                double spanS = maxS - minS;
                double spanT = targetMax - targetMin;
                if (spanS > 1e-12 && spanT > 1e-12)
                {
                    for (int i = 0; i < nV; i++)
                        uSmooth[i] = targetMin + ((uSmooth[i] - minS) / spanS) * spanT;
                }
            }

            mesh.ComputeGradient(uSmooth, out faceGradients, out _);
            coloredMesh = ColorMeshByScalar(mesh.RhinoMesh, uSmooth, false);
            return uSmooth;
        }

        /// <summary>
        /// Smooths any noisy vector, line (2-RoSy), or cross (4-RoSy) direction field on a mesh using
        /// Intrinsic Connection Laplacian Diffusion (Vector Heat Method / N-RoSy parallel transport).
        /// </summary>
        public static Vector3d[] SmoothNRoSyVectorField(
            DdgMesh mesh,
            IList<Vector3d> inputVectors,
            double smoothness,
            int symmetryDegree,
            out Vector3d[] smoothedFaceVectors,
            out Vector3d[] orthogonalVertexVectors,
            out List<Line> previewLines)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;
            int nSym = symmetryDegree <= 1 ? 1 : (symmetryDegree >= 4 ? 4 : 2);

            Vector3d[] vRaw = mesh.MapVectorInputToVertices(inputVectors);
            double h = mesh.MeanEdgeLength;
            double t = Math.Max(0.01, smoothness) * h * h;

            // Build 2*nV real block system for (M + t * L_conn^(nSym)) psi = M * psi_0
            var builder = new SparseMatrixBuilder(2 * nV, 2 * nV);
            double[] rhs = new double[2 * nV];

            // Compute median magnitude so noisy high-curvature spikes don't dominate the direction field
            double maxMag = 1e-12;
            for (int i = 0; i < nV; i++)
            {
                Vector3d proj = vRaw[i] - (vRaw[i] * mesh.VertexNormals[i]) * mesh.VertexNormals[i];
                if (proj.Length > maxMag) maxMag = proj.Length;
            }

            for (int i = 0; i < nV; i++)
            {
                double mass = mesh.DualAreas[i];
                builder.Add(2 * i, 2 * i, mass);
                builder.Add(2 * i + 1, 2 * i + 1, mass);

                Vector3d vProj = vRaw[i] - (vRaw[i] * mesh.VertexNormals[i]) * mesh.VertexNormals[i];
                double len = vProj.Length;
                if (len > 1e-10)
                {
                    vProj /= len;
                    // Soft-normalized weight so all vertices with valid directions contribute smoothly
                    double weight = Math.Pow(len / maxMag, 0.25);
                    double vx = vProj * mesh.TangentBasisX[i];
                    double vy = vProj * mesh.TangentBasisY[i];
                    double theta = Math.Atan2(vy, vx);
                    rhs[2 * i] = mass * weight * Math.Cos(nSym * theta);
                    rhs[2 * i + 1] = mass * weight * Math.Sin(nSym * theta);
                }
            }

            foreach (var edge in mesh.Edges)
            {
                int i = edge.V0;
                int j = edge.V1;
                double w = Math.Max(1e-6, edge.CotanWeight) * t;

                double thetaIJ = FindHalfEdgeAngle(mesh, i, j);
                double thetaJI = FindHalfEdgeAngle(mesh, j, i);
                double rhoIJ = nSym * (thetaIJ - (thetaJI + Math.PI));
                double c = Math.Cos(rhoIJ);
                double s = Math.Sin(rhoIJ);

                builder.Add(2 * i, 2 * i, w);
                builder.Add(2 * i + 1, 2 * i + 1, w);
                builder.Add(2 * j, 2 * j, w);
                builder.Add(2 * j + 1, 2 * j + 1, w);

                builder.Add(2 * i, 2 * j, -w * c);
                builder.Add(2 * i, 2 * j + 1, w * s);
                builder.Add(2 * i + 1, 2 * j, -w * s);
                builder.Add(2 * i + 1, 2 * j + 1, -w * c);

                builder.Add(2 * j, 2 * i, -w * c);
                builder.Add(2 * j, 2 * i + 1, -w * s);
                builder.Add(2 * j + 1, 2 * i, w * s);
                builder.Add(2 * j + 1, 2 * i + 1, -w * c);
            }

            double[] u = SparseLinearSolver.SolvePCG(builder.Build(), rhs, 1200, 1e-8);

            var smoothedVertVectors = new Vector3d[nV];
            orthogonalVertexVectors = new Vector3d[nV];
            previewLines = new List<Line>(nV * (nSym == 4 ? 2 : 1));
            double tickScale = 0.35 * h;

            for (int i = 0; i < nV; i++)
            {
                double re = u[2 * i];
                double im = u[2 * i + 1];
                double phi = Math.Atan2(im, re) / nSym;
                Vector3d dir = Math.Cos(phi) * mesh.TangentBasisX[i] + Math.Sin(phi) * mesh.TangentBasisY[i];
                if (dir.Length > 1e-12) dir.Unitize();
                Vector3d ortho = Vector3d.CrossProduct(mesh.VertexNormals[i], dir);
                if (ortho.Length > 1e-12) ortho.Unitize();

                smoothedVertVectors[i] = dir;
                orthogonalVertexVectors[i] = ortho;

                Point3d p = mesh.Positions[i];
                if (nSym == 1)
                {
                    previewLines.Add(new Line(p, p + dir * (tickScale * 1.4)));
                }
                else
                {
                    previewLines.Add(new Line(p - dir * tickScale, p + dir * tickScale));
                    if (nSym == 4)
                        previewLines.Add(new Line(p - ortho * tickScale, p + ortho * tickScale));
                }
            }

            // Project smoothed N-RoSy field consistently onto triangle faces for Streamlines
            smoothedFaceVectors = new Vector3d[nF];
            for (int f = 0; f < nF; f++)
            {
                int[] fv = mesh.Faces[f];
                Vector3d nf = mesh.FaceNormals[f];
                Vector3d refDir = smoothedVertVectors[fv[0]];
                refDir -= (refDir * nf) * nf;
                if (refDir.Length > 1e-12) refDir.Unitize();

                Vector3d accum = refDir;
                for (int k = 1; k < 3; k++)
                {
                    Vector3d vk = smoothedVertVectors[fv[k]];
                    vk -= (vk * nf) * nf;
                    if (vk.Length > 1e-12) vk.Unitize();

                    if (nSym == 2)
                    {
                        if (vk * refDir < 0) vk = -vk;
                    }
                    else if (nSym == 4)
                    {
                        Vector3d refOrtho = Vector3d.CrossProduct(nf, refDir);
                        double d0 = vk * refDir;
                        double d1 = vk * refOrtho;
                        if (Math.Abs(d0) >= Math.Abs(d1))
                            vk = d0 >= 0 ? vk : -vk;
                        else
                            vk = d1 >= 0 ? Vector3d.CrossProduct( vk, nf) : Vector3d.CrossProduct(nf, vk);
                    }
                    accum += vk;
                }
                accum -= (accum * nf) * nf;
                if (accum.Length > 1e-12) accum.Unitize();
                smoothedFaceVectors[f] = accum;
            }

            return smoothedVertVectors;
        }

        // =========================================================================================
        // HELPER UTILITIES
        // =========================================================================================
        private static double FindHalfEdgeAngle(DdgMesh mesh, int originV, int targetV)
        {
            foreach (int heIdx in mesh.VertexOutgoingHalfEdges[originV])
            {
                if (mesh.HalfEdges[heIdx].TargetVertex == targetV)
                    return mesh.HalfEdges[heIdx].AngularCoordinate;
            }
            return 0.0;
        }

        private static double WrapPi(double angle)
        {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle <= -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }

        private static void AddQuaternionBlock(SparseMatrixBuilder builder, int rowBlk, int colBlk, double w, double x, double y, double z)
        {
            int r = 4 * rowBlk;
            int c = 4 * colBlk;
            builder.Add(r + 0, c + 0, w);  builder.Add(r + 0, c + 1, -x); builder.Add(r + 0, c + 2, -y); builder.Add(r + 0, c + 3, -z);
            builder.Add(r + 1, c + 0, x);  builder.Add(r + 1, c + 1, w);  builder.Add(r + 1, c + 2, -z); builder.Add(r + 1, c + 3, y);
            builder.Add(r + 2, c + 0, y);  builder.Add(r + 2, c + 1, z);  builder.Add(r + 2, c + 2, w);  builder.Add(r + 2, c + 3, -x);
            builder.Add(r + 3, c + 0, z);  builder.Add(r + 3, c + 1, -y); builder.Add(r + 3, c + 2, x);  builder.Add(r + 3, c + 3, w);
        }

        private static double[] GetQuat(double[] spinVec, int vIdx)
        {
            return new double[] { spinVec[4 * vIdx], spinVec[4 * vIdx + 1], spinVec[4 * vIdx + 2], spinVec[4 * vIdx + 3] };
        }

        private static Vector3d SandwichQuat(double[] l1, Vector3d e, double[] l2)
        {
            double[] c1 = new double[] { l1[0], -l1[1], -l1[2], -l1[3] };
            double[] qe = new double[] { 0.0, e.X, e.Y, e.Z };
            double[] m = MulQuat(c1, qe);
            double[] res = MulQuat(m, l2);
            return new Vector3d(res[1], res[2], res[3]);
        }

        private static double[] MulQuat(double[] a, double[] b)
        {
            return new double[]
            {
                a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
            };
        }

        private static List<int> ExtractOrderedBoundaryLoop(DdgMesh mesh)
        {
            var nextBoundaryVertex = new Dictionary<int, int>();
            for (int i = 0; i < mesh.HalfEdges.Count; i++)
            {
                var he = mesh.HalfEdges[i];
                if (he.Twin == -1)
                    nextBoundaryVertex[he.OriginVertex] = he.TargetVertex;
            }
            var loop = new List<int>();
            if (nextBoundaryVertex.Count == 0) return loop;

            int start = -1;
            foreach (var k in nextBoundaryVertex.Keys) { start = k; break; }
            int curr = start;
            var visited = new HashSet<int>();
            while (curr != -1 && !visited.Contains(curr))
            {
                visited.Add(curr);
                loop.Add(curr);
                if (!nextBoundaryVertex.TryGetValue(curr, out curr))
                    break;
            }
            return loop;
        }

        /// <summary>
        /// Colors mesh vertices using Keenan Crane's Heat / Polyscope scientific colormap.
        /// </summary>
        public static Mesh ColorMeshByScalar(Mesh baseMesh, double[] values, bool cyclicStripes = false)
        {
            Mesh m = baseMesh.DuplicateMesh();
            int n = m.Vertices.Count;
            m.VertexColors.Clear();
            if (values == null || values.Length == 0) return m;

            double minV = double.MaxValue, maxV = double.MinValue;
            for (int i = 0; i < Math.Min(n, values.Length); i++)
            {
                if (values[i] < minV) minV = values[i];
                if (values[i] > maxV) maxV = values[i];
            }
            double span = Math.Max(1e-12, maxV - minV);

            for (int i = 0; i < n; i++)
            {
                double v = values[Math.Min(i, values.Length - 1)];
                double t = (v - minV) / span;
                if (cyclicStripes)
                {
                    int c = v >= 0.0 ? 242 : 28;
                    m.VertexColors.Add(Color.FromArgb(c, c, c));
                }
                else
                {
                    // Keenan Crane Heat Method Palette: White-Yellow (t=0) -> Orange -> Crimson Red -> Deep Burgundy (t=1)
                    int r, g, b;
                    if (t < 0.25)
                    {
                        double s = t / 0.25;
                        r = 255;
                        g = (int)(255 - 35 * s);
                        b = (int)(245 - 185 * s);
                    }
                    else if (t < 0.65)
                    {
                        double s = (t - 0.25) / 0.40;
                        r = (int)(255 - 45 * s);
                        g = (int)(220 - 165 * s);
                        b = (int)(60 - 40 * s);
                    }
                    else
                    {
                        double s = (t - 0.65) / 0.35;
                        r = (int)(210 - 125 * s);
                        g = (int)(55 - 40 * s);
                        b = (int)(20 + 10 * s);
                    }
                    m.VertexColors.Add(Color.FromArgb(
                        Math.Max(0, Math.Min(255, r)),
                        Math.Max(0, Math.Min(255, g)),
                        Math.Max(0, Math.Min(255, b))));
                }
            }
            return m;
        }
    }
}
