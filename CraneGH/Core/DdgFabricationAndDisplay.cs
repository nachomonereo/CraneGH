using System;
using System.Collections.Generic;
using System.Drawing;
using Rhino.Geometry;

namespace CraneGH.Core
{
    public static partial class DdgAlgorithms
    {
        // =========================================================================================
        // 13. EXACT ISO-CUT & BAND SLICER (Real Scalar Units or Normalized [0..1])
        // =========================================================================================
        public static List<Polyline> ComputeExactIsoCutAndBands(
            DdgMesh mesh,
            IList<double> rawScalars,
            IList<double> isoValuesInput,
            bool normalizedInput,
            out Mesh lowerMesh,
            out Mesh upperMesh,
            out List<Mesh> bandMeshes,
            out Mesh coloredMesh,
            out List<double> resolvedIsoValues)
        {
            double[] u = mesh.MapScalarInputToVertices(rawScalars);
            int nV = mesh.VertexCount;

            double minU = double.MaxValue, maxU = double.MinValue;
            for (int i = 0; i < nV; i++)
            {
                if (u[i] < minU) minU = u[i];
                if (u[i] > maxU) maxU = u[i];
            }
            if (minU > maxU) { minU = 0.0; maxU = 1.0; }
            double span = Math.Max(1e-12, maxU - minU);

            resolvedIsoValues = new List<double>();
            if (isoValuesInput != null && isoValuesInput.Count > 0)
            {
                foreach (double val in isoValuesInput)
                {
                    double realIso = normalizedInput ? (minU + val * span) : val;
                    resolvedIsoValues.Add(realIso);
                }
            }
            else
            {
                resolvedIsoValues.Add(minU + 0.5 * span);
            }
            resolvedIsoValues.Sort();

            List<Polyline> isoCurves = mesh.ExtractIsoContours(u, resolvedIsoValues);

            double firstIso = resolvedIsoValues[0];
            double lastIso = resolvedIsoValues[resolvedIsoValues.Count - 1];

            lowerMesh = SliceMeshHalfSpace(mesh, u, firstIso, true);
            upperMesh = SliceMeshHalfSpace(mesh, u, lastIso, false);

            bandMeshes = new List<Mesh>();
            var bounds = new List<double> { minU - Math.Abs(span) * 0.1 };
            bounds.AddRange(resolvedIsoValues);
            bounds.Add(maxU + Math.Abs(span) * 0.1);

            Color[] bandPalette = GenerateDistinctPalette(bounds.Count - 1);
            for (int b = 0; b < bounds.Count - 1; b++)
            {
                double lo = bounds[b];
                double hi = bounds[b + 1];
                if (hi <= lo + 1e-12) continue;

                Mesh loClipped = SliceMeshHalfSpace(mesh, u, lo, false);
                if (loClipped.Faces.Count == 0) continue;

                var ddgBand = new DdgMesh(loClipped);
                double[] uBand = new double[ddgBand.VertexCount];
                for (int i = 0; i < ddgBand.VertexCount; i++)
                {
                    Point3d p = ddgBand.Positions[i];
                    MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(p, 0.0);
                    if (mp != null)
                    {
                        MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                        uBand[i] = mp.T[0] * u[mf.A] + mp.T[1] * u[mf.B] + mp.T[2] * u[mf.C];
                    }
                }
                Mesh band = SliceMeshHalfSpace(ddgBand, uBand, hi, true);
                if (band.Faces.Count > 0)
                {
                    band.VertexColors.Clear();
                    for (int vi = 0; vi < band.Vertices.Count; vi++)
                        band.VertexColors.Add(bandPalette[b]);
                    bandMeshes.Add(band);
                }
            }

            coloredMesh = ColorMeshByScalar(mesh.RhinoMesh, u, false);
            return isoCurves;
        }

        public static Mesh SliceMeshHalfSpace(DdgMesh mesh, double[] u, double isoValue, bool keepBelowOrEqual)
        {
            var outMesh = new Mesh();
            double sign = keepBelowOrEqual ? 1.0 : -1.0;

            for (int f = 0; f < mesh.FaceCount; f++)
            {
                int i0 = mesh.Faces[f][0];
                int i1 = mesh.Faces[f][1];
                int i2 = mesh.Faces[f][2];

                Point3d p0 = mesh.Positions[i0];
                Point3d p1 = mesh.Positions[i1];
                Point3d p2 = mesh.Positions[i2];

                double d0 = sign * (u[i0] - isoValue);
                double d1 = sign * (u[i1] - isoValue);
                double d2 = sign * (u[i2] - isoValue);

                var poly = ClipTriangleAgainstZero(p0, p1, p2, d0, d1, d2);
                if (poly.Count == 3)
                {
                    int a = outMesh.Vertices.Add(poly[0]);
                    int b = outMesh.Vertices.Add(poly[1]);
                    int c = outMesh.Vertices.Add(poly[2]);
                    outMesh.Faces.AddFace(a, b, c);
                }
                else if (poly.Count == 4)
                {
                    int a = outMesh.Vertices.Add(poly[0]);
                    int b = outMesh.Vertices.Add(poly[1]);
                    int c = outMesh.Vertices.Add(poly[2]);
                    int d = outMesh.Vertices.Add(poly[3]);
                    outMesh.Faces.AddFace(a, b, c);
                    outMesh.Faces.AddFace(a, c, d);
                }
            }

            outMesh.Normals.ComputeNormals();
            return outMesh;
        }

        private static List<Point3d> ClipTriangleAgainstZero(
            Point3d p0, Point3d p1, Point3d p2,
            double d0, double d1, double d2)
        {
            var pts = new Point3d[] { p0, p1, p2 };
            var ds = new double[] { d0, d1, d2 };
            var res = new List<Point3d>(4);

            for (int i = 0; i < 3; i++)
            {
                int j = (i + 1) % 3;
                Point3d A = pts[i];
                Point3d B = pts[j];
                double dA = ds[i];
                double dB = ds[j];

                bool inA = dA <= 1e-12;
                bool inB = dB <= 1e-12;

                if (inA)
                    res.Add(A);

                if (inA != inB)
                {
                    double denom = dA - dB;
                    double t = Math.Abs(denom) > 1e-14 ? Math.Max(0.0, Math.Min(1.0, dA / denom)) : 0.5;
                    res.Add(A + t * (B - A));
                }
            }
            return res;
        }

        // =========================================================================================
        // 14. WALK ON MESH & SCALAR GRADIENT INTEGRATOR
        // =========================================================================================
        public static List<Polyline> ComputeWalkOnMesh(
            DdgMesh mesh,
            IList<double> rawScalarField,
            IList<Point3d> startPoints,
            double stepSize,
            int maxSteps,
            int walkMode, // 0 = Steepest Descent (-grad u), 1 = Steepest Ascent (+grad u), 2 = Iso-Level Walk (ortho), 3 = Stochastic Geodesic Walk
            out Vector3d[] vertexGradients,
            out Vector3d[] faceGradients,
            out List<List<double>> walkScalarHistory)
        {
            double[] u = (rawScalarField != null && rawScalarField.Count > 0)
                ? mesh.MapScalarInputToVertices(rawScalarField)
                : ComputeHeatDistance(mesh, new int[] { 0 }, 1.0, out _, out _);

            mesh.ComputeGradient(u, out faceGradients, out vertexGradients);

            double hStep = stepSize > 1e-6 ? stepSize : 0.45 * mesh.MeanEdgeLength;
            int nSteps = Math.Max(2, Math.Min(5000, maxSteps));

            var paths = new List<Polyline>();
            walkScalarHistory = new List<List<double>>();
            var rng = new Random(1337);

            if (startPoints == null || startPoints.Count == 0)
                return paths;

            foreach (Point3d seed in startPoints)
            {
                MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(seed, 0.0);
                if (mp == null) continue;

                var pts = new List<Point3d> { mp.Point };
                var scalars = new List<double>();
                Point3d curr = mp.Point;

                for (int step = 0; step < nSteps; step++)
                {
                    MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                    double valCurr = mp.T[0] * u[mf.A] + mp.T[1] * u[mf.B] + mp.T[2] * u[mf.C];
                    scalars.Add(valCurr);

                    Vector3d g = mp.T[0] * vertexGradients[mf.A] +
                                 mp.T[1] * vertexGradients[mf.B] +
                                 mp.T[2] * vertexGradients[mf.C];
                    Vector3d n = mesh.FaceNormals[mp.FaceIndex];
                    g -= (g * n) * n;

                    Vector3d moveDir;
                    if (walkMode == 3)
                    {
                        // Stochastic Brownian / Walk-on-Mesh tangent step with subtle drift
                        double angle = rng.NextDouble() * 2.0 * Math.PI;
                        Vector3d bx = mesh.TangentBasisX[mf.A];
                        Vector3d by = Vector3d.CrossProduct(n, bx);
                        if (by.Length > 1e-12) by.Unitize();
                        moveDir = Math.Cos(angle) * bx + Math.Sin(angle) * by;
                        if (g.Length > 1e-12)
                        {
                            Vector3d gUnit = g / g.Length;
                            moveDir = 0.75 * moveDir - 0.25 * gUnit;
                        }
                    }
                    else
                    {
                        if (g.Length < 1e-12) break;
                        g.Unitize();
                        if (walkMode == 0) moveDir = -g;
                        else if (walkMode == 1) moveDir = g;
                        else moveDir = Vector3d.CrossProduct(n, g);
                    }

                    if (moveDir.Length < 1e-12) break;
                    moveDir.Unitize();

                    Point3d candidate = curr + hStep * moveDir;
                    MeshPoint nextMp = mesh.RhinoMesh.ClosestMeshPoint(candidate, 0.0);
                    if (nextMp == null || nextMp.Point.DistanceToSquared(curr) < 1e-14)
                        break;

                    curr = nextMp.Point;
                    mp = nextMp;
                    pts.Add(curr);
                }

                if (pts.Count >= 2)
                {
                    paths.Add(new Polyline(pts));
                    walkScalarHistory.Add(scalars);
                }
            }

            return paths;
        }

        // =========================================================================================
        // 15. DISCRETIZE VECTOR FIELD WITH A FAMILY OF VECTORS ON MESH
        // =========================================================================================
        public static Vector3d[] DiscretizeVectorFieldByFamily(
            DdgMesh mesh,
            IList<Vector3d> continuousField,
            IList<Vector3d> customVectorFamily,
            int familyCountK,
            bool symmetricRose,
            out Vector3d[] discretizedVertexVectors,
            out Vector3d[] canonicalFamilyDirections,
            out int[] faceFamilyIds,
            out List<Mesh> familySubMeshes,
            out Mesh coloredMesh)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;

            Vector3d[] vField = (continuousField != null && continuousField.Count > 0)
                ? mesh.MapVectorInputToVertices(continuousField)
                : mesh.PrincipalDir;

            // Convert vertex field to face tangent field
            var fField = new Vector3d[nF];
            for (int f = 0; f < nF; f++)
            {
                if (continuousField != null && continuousField.Count == nF && nF != nV)
                {
                    fField[f] = continuousField[f];
                }
                else
                {
                    int[] fv = mesh.Faces[f];
                    fField[f] = (vField[fv[0]] + vField[fv[1]] + vField[fv[2]]) / 3.0;
                }
                fField[f] -= (fField[f] * mesh.FaceNormals[f]) * mesh.FaceNormals[f];
                if (fField[f].Length > 1e-12) fField[f].Unitize();
                else fField[f] = Vector3d.XAxis;
            }

            // Build discrete family of vectors
            int K;
            if (customVectorFamily != null && customVectorFamily.Count > 0)
            {
                var validList = new List<Vector3d>();
                foreach (var v in customVectorFamily)
                {
                    if (v.IsValid && v.Length > 1e-12)
                    {
                        Vector3d u = v;
                        u.Unitize();
                        validList.Add(u);
                    }
                }
                if (validList.Count == 0) validList.Add(Vector3d.XAxis);
                canonicalFamilyDirections = validList.ToArray();
                K = canonicalFamilyDirections.Length;
            }
            else
            {
                K = Math.Max(2, Math.Min(32, familyCountK));
                canonicalFamilyDirections = new Vector3d[K];

                // Initialize K directions via K-Means++ on face tangent vectors
                var rng = new Random(42);
                canonicalFamilyDirections[0] = fField[0];
                double[] minDist = new double[nF];
                for (int f = 0; f < nF; f++) minDist[f] = double.MaxValue;

                for (int c = 1; c < K; c++)
                {
                    int bestF = 0;
                    double maxD = -1.0;
                    for (int f = 0; f < nF; f++)
                    {
                        double dot = fField[f] * canonicalFamilyDirections[c - 1];
                        double align = symmetricRose ? Math.Abs(dot) : dot;
                        double d = 1.0 - Math.Max(-1.0, Math.Min(1.0, align));
                        if (d < minDist[f]) minDist[f] = d;
                        if (minDist[f] > maxD) { maxD = minDist[f]; bestF = f; }
                    }
                    canonicalFamilyDirections[c] = fField[bestF];
                }

                // 8 iterations of spherical K-Means on tangent directions
                int[] assign = new int[nF];
                for (int iter = 0; iter < 8; iter++)
                {
                    var sumVecs = new Vector3d[K];
                    for (int f = 0; f < nF; f++)
                    {
                        int bestK = 0;
                        double bestDot = -2.0;
                        for (int k = 0; k < K; k++)
                        {
                            double dot = fField[f] * canonicalFamilyDirections[k];
                            double score = symmetricRose ? Math.Abs(dot) : dot;
                            if (score > bestDot) { bestDot = score; bestK = k; }
                        }
                        assign[f] = bestK;
                        Vector3d aligned = (symmetricRose && (fField[f] * canonicalFamilyDirections[bestK] < 0))
                            ? -fField[f]
                            : fField[f];
                        sumVecs[bestK] += aligned * mesh.FaceAreas[f];
                    }
                    for (int k = 0; k < K; k++)
                    {
                        if (sumVecs[k].Length > 1e-12)
                        {
                            sumVecs[k].Unitize();
                            canonicalFamilyDirections[k] = sumVecs[k];
                        }
                    }
                }
            }

            // Snap each face vector to the closest family member (projected onto face tangent plane)
            faceFamilyIds = new int[nF];
            var discFaceVecs = new Vector3d[nF];
            for (int f = 0; f < nF; f++)
            {
                Vector3d nf = mesh.FaceNormals[f];
                int bestK = 0;
                double bestScore = -2.0;
                Vector3d bestTan = fField[f];

                for (int k = 0; k < K; k++)
                {
                    Vector3d famProj = canonicalFamilyDirections[k] - (canonicalFamilyDirections[k] * nf) * nf;
                    if (famProj.Length < 1e-10) continue;
                    famProj.Unitize();

                    double dot = fField[f] * famProj;
                    double score = symmetricRose ? Math.Abs(dot) : dot;
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestK = k;
                        bestTan = (symmetricRose && dot < 0) ? -famProj : famProj;
                    }
                }
                faceFamilyIds[f] = bestK;
                discFaceVecs[f] = bestTan;
            }

            // 1-pass majority vote smoothing on faceFamilyIds to eliminate isolated 1-triangle noise
            int[] smoothedIds = (int[])faceFamilyIds.Clone();
            for (int f = 0; f < nF; f++)
            {
                int c0 = faceFamilyIds[f];
                int sameCount = 0;
                int neighborMode = c0;
                for (int k = 0; k < 3; k++)
                {
                    var he = mesh.HalfEdges[f * 3 + k];
                    if (he.Twin != -1)
                    {
                        int nfIdx = mesh.HalfEdges[he.Twin].FaceIndex;
                        if (faceFamilyIds[nfIdx] == c0) sameCount++;
                        else neighborMode = faceFamilyIds[nfIdx];
                    }
                }
                if (sameCount == 0) smoothedIds[f] = neighborMode;
            }
            faceFamilyIds = smoothedIds;

            // Discretize vertex vectors similarly
            discretizedVertexVectors = new Vector3d[nV];
            for (int v = 0; v < nV; v++)
            {
                Vector3d nv = mesh.VertexNormals[v];
                int bestK = 0;
                double bestScore = -2.0;
                Vector3d bestTan = vField[v];
                for (int k = 0; k < K; k++)
                {
                    Vector3d famProj = canonicalFamilyDirections[k] - (canonicalFamilyDirections[k] * nv) * nv;
                    if (famProj.Length < 1e-10) continue;
                    famProj.Unitize();
                    double dot = vField[v] * famProj;
                    double score = symmetricRose ? Math.Abs(dot) : dot;
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestK = k;
                        bestTan = (symmetricRose && dot < 0) ? -famProj : famProj;
                    }
                }
                discretizedVertexVectors[v] = bestTan;
            }

            // Build segmented sub-meshes by family ID
            Color[] palette = GenerateDistinctPalette(K);
            familySubMeshes = BuildSubMeshesFromFaceLabels(mesh, faceFamilyIds, K, palette, out coloredMesh);

            return discFaceVecs;
        }

        // =========================================================================================
        // 16. K-MEANS CLUSTERING ON MESH FACE NORMALS + VECTOR FIELD + FEATURES (VSA / LLOYD)
        // =========================================================================================
        public static List<Mesh> ComputeKMeansNormalVectorSegmentation(
            DdgMesh mesh,
            int clusterCountK,
            double normalWeight,
            double positionWeight,
            double vectorFieldWeight,
            double creaseFeatureWeight,
            IList<Vector3d> optionalVectorField,
            IList<double> optionalFeatureScalar,
            out Vector3d[] clusterPullNormals,
            out Point3d[] clusterCentroids,
            out List<Polyline> boundaryCurves,
            out int[] faceLabels,
            out Mesh coloredMesh)
        {
            int nF = mesh.FaceCount;
            int K = Math.Max(2, Math.Min(nF, clusterCountK));

            // Normalize mesh bounding box diagonal so position distance is scale-invariant in [0, 1]
            BoundingBox bbox = mesh.RhinoMesh.GetBoundingBox(false);
            double invDiag = 1.0 / Math.Max(1e-6, bbox.Diagonal.Length);

            // Map optional vector field to faces
            var faceVecs = new Vector3d[nF];
            bool hasVecField = optionalVectorField != null && optionalVectorField.Count > 0 && vectorFieldWeight > 1e-6;
            if (hasVecField)
            {
                if (optionalVectorField.Count == nF && nF != mesh.VertexCount)
                {
                    for (int f = 0; f < nF; f++)
                    {
                        Vector3d v = optionalVectorField[f];
                        if (v.Length > 1e-12) v.Unitize();
                        faceVecs[f] = v;
                    }
                }
                else
                {
                    Vector3d[] vMapped = mesh.MapVectorInputToVertices(optionalVectorField);
                    for (int f = 0; f < nF; f++)
                    {
                        int[] fv = mesh.Faces[f];
                        Vector3d v = vMapped[fv[0]] + vMapped[fv[1]] + vMapped[fv[2]];
                        if (v.Length > 1e-12) v.Unitize();
                        faceVecs[f] = v;
                    }
                }
            }

            // Map optional feature scalar (or default vertex sharpness + Gaussian curvature) to faces in [0, 1]
            double[] faceFeat = new double[nF];
            double[] vertFeat = (optionalFeatureScalar != null && optionalFeatureScalar.Count > 0)
                ? mesh.MapScalarInputToVertices(optionalFeatureScalar)
                : mesh.VertexSharpness;

            double minF = double.MaxValue, maxF = double.MinValue;
            for (int i = 0; i < vertFeat.Length; i++)
            {
                if (vertFeat[i] < minF) minF = vertFeat[i];
                if (vertFeat[i] > maxF) maxF = vertFeat[i];
            }
            double spanF = Math.Max(1e-12, maxF - minF);
            for (int f = 0; f < nF; f++)
            {
                int[] fv = mesh.Faces[f];
                double avg = (vertFeat[fv[0]] + vertFeat[fv[1]] + vertFeat[fv[2]]) / 3.0;
                faceFeat[f] = (avg - minF) / spanF;
            }

            // 1. K-Means++ initialization in combined (Position, Normal, VectorField, Feature) metric
            clusterCentroids = new Point3d[K];
            clusterPullNormals = new Vector3d[K];
            var clusterVecs = new Vector3d[K];
            var clusterFeats = new double[K];

            int firstSeed = 0;
            clusterCentroids[0] = mesh.FaceCenters[firstSeed];
            clusterPullNormals[0] = mesh.FaceNormals[firstSeed];
            clusterVecs[0] = faceVecs[firstSeed];
            clusterFeats[0] = faceFeat[firstSeed];

            double[] minCostToSeeds = new double[nF];
            for (int f = 0; f < nF; f++) minCostToSeeds[f] = double.MaxValue;

            for (int c = 1; c < K; c++)
            {
                int prev = c - 1;
                int farthestFace = 0;
                double maxD = -1.0;

                for (int f = 0; f < nF; f++)
                {
                    double d = EvaluateKMeansFaceCost(
                        mesh.FaceCenters[f], mesh.FaceNormals[f], faceVecs[f], faceFeat[f],
                        clusterCentroids[prev], clusterPullNormals[prev], clusterVecs[prev], clusterFeats[prev],
                        invDiag, normalWeight, positionWeight, hasVecField ? vectorFieldWeight : 0.0, creaseFeatureWeight);

                    if (d < minCostToSeeds[f]) minCostToSeeds[f] = d;
                    if (minCostToSeeds[f] > maxD)
                    {
                        maxD = minCostToSeeds[f];
                        farthestFace = f;
                    }
                }

                clusterCentroids[c] = mesh.FaceCenters[farthestFace];
                clusterPullNormals[c] = mesh.FaceNormals[farthestFace];
                clusterVecs[c] = faceVecs[farthestFace];
                clusterFeats[c] = faceFeat[farthestFace];
            }

            // 2. Lloyd K-Means iterations + Geodesic-Crease Flood-Fill refinement (Variational Shape Approximation)
            faceLabels = new int[nF];
            for (int iter = 0; iter < 12; iter++)
            {
                var sumPos = new Vector3d[K];
                var sumNorm = new Vector3d[K];
                var sumVec = new Vector3d[K];
                var sumFeat = new double[K];
                var sumArea = new double[K];

                for (int f = 0; f < nF; f++)
                {
                    int bestC = 0;
                    double bestCost = double.MaxValue;
                    for (int c = 0; c < K; c++)
                    {
                        double cost = EvaluateKMeansFaceCost(
                            mesh.FaceCenters[f], mesh.FaceNormals[f], faceVecs[f], faceFeat[f],
                            clusterCentroids[c], clusterPullNormals[c], clusterVecs[c], clusterFeats[c],
                            invDiag, normalWeight, positionWeight, hasVecField ? vectorFieldWeight : 0.0, creaseFeatureWeight);

                        if (cost < bestCost)
                        {
                            bestCost = cost;
                            bestC = c;
                        }
                    }
                    faceLabels[f] = bestC;
                    double a = mesh.FaceAreas[f];
                    sumPos[bestC] += new Vector3d(mesh.FaceCenters[f]) * a;
                    sumNorm[bestC] += mesh.FaceNormals[f] * a;
                    if (hasVecField) sumVec[bestC] += faceVecs[f] * a;
                    sumFeat[bestC] += faceFeat[f] * a;
                    sumArea[bestC] += a;
                }

                for (int c = 0; c < K; c++)
                {
                    if (sumArea[c] <= 1e-12) continue;
                    clusterCentroids[c] = new Point3d(sumPos[c] / sumArea[c]);
                    Vector3d n = sumNorm[c];
                    if (n.Length > 1e-12) n.Unitize();
                    clusterPullNormals[c] = n;

                    if (hasVecField && sumVec[c].Length > 1e-12)
                    {
                        Vector3d v = sumVec[c];
                        v.Unitize();
                        clusterVecs[c] = v;
                    }
                    clusterFeats[c] = sumFeat[c] / sumArea[c];
                }
            }

            // 3. Crease-locked boundary smoothing pass so cluster borders snap to sharp edges
            for (int pass = 0; pass < 2; pass++)
            {
                int[] nextLabels = (int[])faceLabels.Clone();
                for (int f = 0; f < nF; f++)
                {
                    var votes = new Dictionary<int, double>();
                    votes[faceLabels[f]] = 0.6;
                    for (int k = 0; k < 3; k++)
                    {
                        var he = mesh.HalfEdges[f * 3 + k];
                        if (he.Twin == -1) continue;
                        int nfIdx = mesh.HalfEdges[he.Twin].FaceIndex;
                        var edge = mesh.Edges[he.EdgeIndex];
                        // Strong dihedral crease prevents neighbor vote from crossing the crease
                        double creaseAtten = Math.Exp(-creaseFeatureWeight * 3.0 * edge.DihedralAngle);
                        int nbLabel = faceLabels[nfIdx];
                        if (votes.TryGetValue(nbLabel, out double cur))
                            votes[nbLabel] = cur + creaseAtten;
                        else
                            votes[nbLabel] = creaseAtten;
                    }
                    int bestL = faceLabels[f];
                    double maxV = -1.0;
                    foreach (var kv in votes)
                    {
                        if (kv.Value > maxV) { maxV = kv.Value; bestL = kv.Key; }
                    }
                    nextLabels[f] = bestL;
                }
                faceLabels = nextLabels;
            }

            EnforceConnectedClusterRegions(mesh, faceLabels, K);

            Color[] palette = GenerateDistinctPalette(K);
            List<Mesh> subMeshes = BuildSubMeshesFromFaceLabels(mesh, faceLabels, K, palette, out coloredMesh);

            List<Polyline> rawBoundaryCurves = ExtractLabelBoundaryCurves(mesh, faceLabels);
            if (rawBoundaryCurves.Count > 0)
            {
                // Stretch & shorten cluster boundaries into taut Geodesic Curves on the mesh (FlipOut / Geodesic Network Shorten)
                var seamCurves = rawBoundaryCurves.ConvertAll(pl => (Curve)new PolylineCurve(pl));
                boundaryCurves = ComputeGeodesicNetworkShorten(
                    mesh.RhinoMesh, seamCurves, 14, true, out _, creaseFeatureWeight > 0.1 ? vertFeat : null);
            }
            else
            {
                boundaryCurves = rawBoundaryCurves;
            }

            return subMeshes;
        }

        private static double EvaluateKMeansFaceCost(
            Point3d pF, Vector3d nF, Vector3d vF, double featF,
            Point3d pC, Vector3d nC, Vector3d vC, double featC,
            double invDiag, double wN, double wP, double wV, double wF)
        {
            double dPos = (pF.DistanceTo(pC) * invDiag);
            double dNorm = 1.0 - Math.Max(-1.0, Math.Min(1.0, nF * nC)); // L2 normal metric ||N_f - N_c||^2 / 2
            double dVec = wV > 0.0 ? (1.0 - Math.Max(-1.0, Math.Min(1.0, vF * vC))) : 0.0;
            double dFeat = Math.Abs(featF - featC);

            return wN * dNorm * dNorm
                 + wP * dPos * dPos
                 + wV * dVec * dVec
                 + wF * dFeat * dFeat;
        }

        private static List<Mesh> BuildSubMeshesFromFaceLabels(
            DdgMesh mesh, int[] faceLabels, int K, Color[] palette, out Mesh coloredCombinedMesh)
        {
            int nF = mesh.FaceCount;
            coloredCombinedMesh = new Mesh();
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
                            sub.VertexColors.Add(palette[c % palette.Length]);
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
                    coloredCombinedMesh.Append(sub);
                }
            }
            return subMeshes;
        }

        private static List<Polyline> ExtractLabelBoundaryCurves(DdgMesh mesh, int[] faceLabels)
        {
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

            var boundaryCurves = new List<Polyline>();
            if (cutLines.Count > 0)
            {
                Curve[] joined = Curve.JoinCurves(cutLines.ConvertAll(l => (Curve)new LineCurve(l)), mesh.MeanEdgeLength * 1e-3);
                if (joined != null)
                {
                    foreach (var crv in joined)
                        if (crv.TryGetPolyline(out Polyline pl))
                            boundaryCurves.Add(pl);
                }
            }
            return boundaryCurves;
        }

        // =========================================================================================
        // 17. ROLLING SPHERES MEDIAL AXIS, CNC ACCESSIBILITY & COLLISION-FREE OFFSET
        //     (Keenan Crane's Tangent-Point Energy / Repulsive Surfaces 2021 + Shrinking Ball)
        // =========================================================================================
        public static double[] ComputeRollingSpheresAndOffset(
            DdgMesh mesh,
            double targetToolOrOffsetRadius,
            bool exteriorSide,
            int maxSampleSpheres,
            out List<Sphere> rollingSpheres,
            out Mesh collisionFreeOffsetMesh,
            out List<Point3d> gougeCollisionPoints,
            out Mesh coloredAccessibilityMesh)
        {
            int nV = mesh.VertexCount;
            double sideSign = exteriorSide ? 1.0 : -1.0;

            BoundingBox bbox = mesh.RhinoMesh.GetBoundingBox(false);
            double maxPossibleR = Math.Max(1e-4, bbox.Diagonal.Length * 0.75);
            double desiredR = Math.Abs(targetToolOrOffsetRadius) > 1e-6
                ? Math.Abs(targetToolOrOffsetRadius)
                : 2.0 * mesh.MeanEdgeLength;

            // Subsample witness points if mesh is very dense (> 1500 vertices) for instant interactive performance
            int stride = Math.Max(1, nV / 1200);
            var sampleIndices = new List<int>(nV / stride + 1);
            for (int i = 0; i < nV; i += stride)
                sampleIndices.Add(i);

            double[] maxRollingRadius = new double[nV];
            gougeCollisionPoints = new List<Point3d>();

            for (int i = 0; i < nV; i++)
            {
                Point3d xi = mesh.Positions[i];
                Vector3d ni = sideSign * mesh.VertexNormals[i];

                // Local curvature limit 1 / kappa_normal
                double rMin = maxPossibleR;

                // 1. Check 1-ring neighbors for immediate normal curvature radius
                foreach (int heIdx in mesh.VertexOutgoingHalfEdges[i])
                {
                    int j = mesh.HalfEdges[heIdx].TargetVertex;
                    Vector3d d = mesh.Positions[j] - xi;
                    double hProj = d * ni;
                    if (hProj > 1e-9)
                    {
                        double rCand = d.SquareLength / (2.0 * hProj);
                        if (rCand < rMin) rMin = rCand;
                    }
                }

                // 2. Check global non-local Tangent-Point sphere condition: R(xi, xj) = ||xj - xi||^2 / (2 * <ni, xj - xi>)
                for (int s = 0; s < sampleIndices.Count; s++)
                {
                    int j = sampleIndices[s];
                    if (i == j) continue;
                    Vector3d d = mesh.Positions[j] - xi;
                    double hProj = d * ni;
                    if (hProj > 1e-7)
                    {
                        double distSq = d.SquareLength;
                        double rCand = distSq / (2.0 * hProj);
                        if (rCand < rMin) rMin = rCand;
                    }
                }

                maxRollingRadius[i] = Math.Max(1e-5, Math.Min(maxPossibleR, rMin));
                if (maxRollingRadius[i] < desiredR)
                {
                    gougeCollisionPoints.Add(xi);
                }
            }

            // Build tangent rolling spheres for viewport visualization & medial axis inspection
            rollingSpheres = new List<Sphere>();
            int sphCount = Math.Max(1, Math.Min(nV, maxSampleSpheres));
            int sphStride = Math.Max(1, nV / sphCount);
            for (int i = 0; i < nV; i += sphStride)
            {
                double r = Math.Min(desiredR, maxRollingRadius[i]);
                Vector3d ni = sideSign * mesh.VertexNormals[i];
                Point3d center = mesh.Positions[i] + r * ni;
                rollingSpheres.Add(new Sphere(center, r));
            }

            // Compute Collision-Free Offset displacement clamped by 0.92 * maxRollingRadius[i] and smoothed via Cotan-Laplace
            double[] clampedOffset = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                clampedOffset[i] = Math.Min(desiredR, 0.92 * maxRollingRadius[i]);
            }

            // Smooth the clamped offset field slightly so the offset surface remains C1-smooth
            double[] smoothOffset = SmoothScalarField(mesh, clampedOffset, 0.5, 0.0, out _, out _);

            collisionFreeOffsetMesh = mesh.RhinoMesh.DuplicateMesh();
            for (int i = 0; i < nV; i++)
            {
                double d = Math.Min( smoothOffset[i], 0.95 * maxRollingRadius[i]);
                Point3d newPt = mesh.Positions[i] + (sideSign * d) * mesh.VertexNormals[i];
                collisionFreeOffsetMesh.Vertices.SetVertex(i, newPt);
            }
            collisionFreeOffsetMesh.Normals.ComputeNormals();

            coloredAccessibilityMesh = ColorMeshByScalar(mesh.RhinoMesh, maxRollingRadius, false);
            return maxRollingRadius;
        }

        // =========================================================================================
        // 18. MOLD PARTING LINES, DRAFT ANGLE & UNDERCUT SEGMENTATION (Fabrication & CAM)
        // =========================================================================================
        public static List<Polyline> ComputeMoldPartingAnalysis(
            DdgMesh mesh,
            Vector3d pullDirection,
            double minDraftAngleDeg,
            double smoothness,
            out double[] draftAngleDegrees,
            out Mesh coreMesh,
            out Mesh cavityMesh,
            out List<Polyline> undercutContours,
            out Mesh coloredDraftMesh)
        {
            int nV = mesh.VertexCount;
            Vector3d pull = pullDirection.Length > 1e-12 ? pullDirection : Vector3d.ZAxis;
            pull.Unitize();

            double[] rawDot = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                rawDot[i] = Math.Max(-1.0, Math.Min(1.0, mesh.VertexNormals[i] * pull));
            }

            double[] smoothDot = smoothness > 1e-5
                ? SmoothScalarField(mesh, rawDot, smoothness, 0.0, out _, out _)
                : rawDot;

            draftAngleDegrees = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                double d = Math.Max(-1.0, Math.Min(1.0, smoothDot[i]));
                // Draft angle in degrees: +90 deg = facing directly up pull direction, 0 deg = vertical wall (parting line), -90 deg = bottom cavity
                draftAngleDegrees[i] = Math.Asin(d) * (180.0 / Math.PI);
            }

            // Exact parting line where N * D = 0
            List<Polyline> partingLines = mesh.ExtractIsoContours(smoothDot, new double[] { 0.0 });

            // Minimum draft warning band at +- minDraftAngleDeg
            double draftSin = Math.Sin(Math.Max(0.1, Math.Abs(minDraftAngleDeg)) * Math.PI / 180.0);
            undercutContours = mesh.ExtractIsoContours(smoothDot, new double[] { -draftSin, draftSin });

            // Split mesh along the exact parting line (smoothDot = 0) into Cavity (dot <= 0) and Core (dot >= 0)
            cavityMesh = SliceMeshHalfSpace(mesh, smoothDot, 0.0, true);
            coreMesh = SliceMeshHalfSpace(mesh, smoothDot, 0.0, false);

            // Color mesh with intuitive Mold Draft Colors:
            // Green/Teal = Positive Draft (Core), Blue = Negative Draft (Cavity), Bright Red/Gold = Insufficient Draft / Undercut zone (|angle| < minDraftAngleDeg)
            coloredDraftMesh = mesh.RhinoMesh.DuplicateMesh();
            coloredDraftMesh.VertexColors.Clear();
            double threshDeg = Math.Max(0.5, Math.Abs(minDraftAngleDeg));

            for (int i = 0; i < nV; i++)
            {
                double deg = draftAngleDegrees[i];
                Color c;
                if (Math.Abs(deg) < threshDeg)
                {
                    // Warning zone near parting line requiring draft taper
                    c = Color.FromArgb(245, 195, 40);
                }
                else if (deg >= threshDeg)
                {
                    double t = Math.Min(1.0, (deg - threshDeg) / (90.0 - threshDeg));
                    c = Color.FromArgb((int)(40 * (1 - t)), (int)(160 + 70 * t), (int)(120 + 40 * t));
                }
                else
                {
                    double t = Math.Min(1.0, (-deg - threshDeg) / (90.0 - threshDeg));
                    c = Color.FromArgb((int)(45 * (1 - t)), (int)(110 + 40 * t), (int)(195 + 55 * t));
                }
                coloredDraftMesh.VertexColors.Add(c);
            }

            return partingLines;
        }

        // =========================================================================================
        // 18B. MULTI-VECTOR, CURVATURE & GEODESIC DRAFT-ANGLE MOLD PARTITIONING (Multi-Piece Molds)
        //      Combines Multiple Pull Vectors + Curvature Ridge Detection + Heat Geodesic Distance
        //      to partition complex 3D meshes into K extractable mold pieces with smooth parting lines.
        // =========================================================================================
        public static List<Mesh> ComputeMultiPieceMoldAndDraftCut(
            DdgMesh mesh,
            IList<Vector3d> inputPullVectors,
            int targetPieceCount,
            double minDraftAngleDeg,
            double geodesicWeight,
            double curvatureSnapWeight,
            double smoothness,
            out Vector3d[] resolvedPullVectors,
            out Point3d[] pieceCentroids,
            out List<Polyline> partingLines,
            out double[] effectiveDraftAnglesDeg,
            out Mesh undercutWarningMesh,
            out int[] faceLabels,
            out Mesh coloredEvaluationMesh)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;
            BoundingBox bbox = mesh.RhinoMesh.GetBoundingBox(false);
            double diagLen = Math.Max(1e-6, bbox.Diagonal.Length);

            // 1. Compute normalized Curvature & Sharp-Crease Ridge Potential [0..1] per vertex and per face
            //    High curvature ridges (|H|, |K|, and dihedral sharpness) are natural mold parting seams!
            double[] rawRidge = new double[nV];
            double maxH = 1e-9, maxSharp = 1e-9;
            for (int i = 0; i < nV; i++)
            {
                double hAbs = Math.Abs(mesh.MeanCurvature[i]);
                double sAbs = mesh.VertexSharpness[i];
                if (hAbs > maxH) maxH = hAbs;
                if (sAbs > maxSharp) maxSharp = sAbs;
            }
            for (int i = 0; i < nV; i++)
            {
                double hNorm = Math.Min(1.0, Math.Abs(mesh.MeanCurvature[i]) / (0.35 * maxH + 1e-9));
                double sNorm = Math.Min(1.0, mesh.VertexSharpness[i] / Math.Max(0.35, maxSharp));
                rawRidge[i] = Math.Min(1.0, 0.55 * sNorm + 0.45 * hNorm);
            }
            double[] ridgeField = SmoothScalarField(mesh, rawRidge, 0.6, 0.0, out _, out _);

            // 2. Clean user-supplied pull vectors
            var userDirs = new List<Vector3d>();
            if (inputPullVectors != null)
            {
                foreach (var v in inputPullVectors)
                {
                    if (v.IsValid && v.Length > 1e-9)
                    {
                        Vector3d u = v;
                        u.Unitize();
                        userDirs.Add(u);
                    }
                }
            }

            // Determine total mold pieces K and initial pull vectors
            int reqK = Math.Max(2, Math.Min(24, targetPieceCount));
            var pullList = new List<Vector3d>();
            var fixedUserDir = new List<bool>();

            if (userDirs.Count >= 2)
            {
                foreach (var d in userDirs)
                {
                    pullList.Add(d);
                    fixedUserDir.Add(true);
                }
            }
            else if (userDirs.Count == 1)
            {
                // Single vector provided: start with +D (Core) and -D (Cavity)
                pullList.Add(userDirs[0]);
                fixedUserDir.Add(true);
                pullList.Add(-userDirs[0]);
                fixedUserDir.Add(true);
            }

            int K = Math.Max(pullList.Count, reqK);
            if (userDirs.Count >= 2 && targetPieceCount <= userDirs.Count)
                K = userDirs.Count;

            // 3. Find Geodesic Anchor Seeds & auto-discover any remaining Pull Vectors via Curvature + Heat Geodesics
            var seedVertices = new List<int>(K);
            var geodesicFields = new List<double[]>(K);

            // First, assign best anchor seed on the mesh for each already-known pull vector
            for (int k = 0; k < pullList.Count; k++)
            {
                Vector3d dk = pullList[k];
                int bestV = 0;
                double bestScore = double.MinValue;
                for (int i = 0; i < nV; i++)
                {
                    double align = mesh.VertexNormals[i] * dk;
                    // Prefer smooth basin interiors with strong alignment and far from existing seeds
                    double minGeo = 1.0;
                    for (int prev = 0; prev < geodesicFields.Count; prev++)
                    {
                        double gNorm = geodesicFields[prev][i] / diagLen;
                        if (gNorm < minGeo) minGeo = gNorm;
                    }
                    double score = align - 0.35 * ridgeField[i] + 0.5 * minGeo;
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestV = i;
                    }
                }
                seedVertices.Add(bestV);
                geodesicFields.Add(ComputeHeatDistance(mesh, new int[] { bestV }, 1.0, out _, out _));
            }

            // Next, if we need to auto-discover pull vectors from Mesh Curvature & Undercut zones (up to K pieces)
            while (pullList.Count < K)
            {
                int bestSeed = 0;
                double maxNeed = double.MinValue;

                for (int i = 0; i < nV; i++)
                {
                    // How well is vertex i currently demolded by existing pull vectors?
                    double bestDraft = -1.0;
                    double minGeoDist = 1.0;
                    for (int m = 0; m < pullList.Count; m++)
                    {
                        double gNorm = geodesicFields[m][i] / diagLen;
                        if (gNorm < minGeoDist) minGeoDist = gNorm;
                        double effAlign = (mesh.VertexNormals[i] * pullList[m]) - 0.25 * gNorm;
                        if (effAlign > bestDraft) bestDraft = effAlign;
                    }

                    // Priority: regions with poor draft (undercuts wrt current vectors), geodesically far from current seeds,
                    // and situated inside a coherent basin (not right on a razor-sharp crease)
                    double undercutNeed = (pullList.Count == 0)
                        ? (1.0 - 0.5 * ridgeField[i])
                        : ((1.0 - bestDraft) * (0.4 + 0.6 * minGeoDist) - 0.30 * ridgeField[i]);

                    if (undercutNeed > maxNeed)
                    {
                        maxNeed = undercutNeed;
                        bestSeed = i;
                    }
                }

                // Compute local basin normal around bestSeed (averaging 2-ring smooth neighborhood)
                Vector3d basinNormal = mesh.VertexNormals[bestSeed] * mesh.DualAreas[bestSeed];
                foreach (int heIdx in mesh.VertexOutgoingHalfEdges[bestSeed])
                {
                    int nb = mesh.HalfEdges[heIdx].TargetVertex;
                    double w = (1.0 - 0.7 * ridgeField[nb]) * mesh.DualAreas[nb];
                    basinNormal += w * mesh.VertexNormals[nb];
                }
                if (basinNormal.Length < 1e-12) basinNormal = Vector3d.ZAxis;
                basinNormal.Unitize();

                pullList.Add(basinNormal);
                fixedUserDir.Add(false);
                seedVertices.Add(bestSeed);
                geodesicFields.Add(ComputeHeatDistance(mesh, new int[] { bestSeed }, 1.0, out _, out _));
            }

            K = pullList.Count;
            resolvedPullVectors = pullList.ToArray();

            // 4. Special exact 2-piece fast path when K == 2 and vectors are opposite (+D / -D) and geodesicWeight == 0
            double wGeo = Math.Max(0.0, geodesicWeight);
            double wCurv = Math.Max(0.0, curvatureSnapWeight);
            double smoothT = Math.Max(0.0, smoothness);

            // 5. Coupled Draft-Angle + Geodesic Distance + Curvature Ridge Competition (2 refinement iterations)
            faceLabels = new int[nF];
            int[] vertexLabels = new int[nV];
            double[][] pieceScores = new double[K][];

            for (int iter = 0; iter < 2; iter++)
            {
                for (int k = 0; k < K; k++)
                {
                    double[] rawScore = new double[nV];
                    Vector3d dk = resolvedPullVectors[k];
                    double[] geoK = geodesicFields[k];

                    for (int i = 0; i < nV; i++)
                    {
                        double draftSin = mesh.VertexNormals[i] * dk; // sin(draftAngle) in [-1, 1]
                        double normGeo = geoK[i] / diagLen;

                        // Curvature crease penalty: penalizes crossing high-curvature ridges as geodesic distance grows
                        double creaseBarrier = wCurv * 0.18 * ridgeField[i] * Math.Min(1.5, normGeo * 3.0);

                        rawScore[i] = draftSin - wGeo * normGeo - creaseBarrier;
                    }

                    // Smooth each piece's compatibility score using Crease-Barrier Cotan-Laplace diffusion
                    // so parting lines snap cleanly to high-curvature creases!
                    pieceScores[k] = smoothT > 1e-5
                        ? SmoothScalarField(mesh, rawScore, smoothT, wCurv * 2.5, out _, out _)
                        : rawScore;
                }

                // Assign vertices and faces to winning mold piece k*
                for (int i = 0; i < nV; i++)
                {
                    int bestK = 0;
                    double bestS = pieceScores[0][i];
                    for (int k = 1; k < K; k++)
                    {
                        if (pieceScores[k][i] > bestS)
                        {
                            bestS = pieceScores[k][i];
                            bestK = k;
                        }
                    }
                    vertexLabels[i] = bestK;
                }

                for (int f = 0; f < nF; f++)
                {
                    int i0 = mesh.Faces[f][0], i1 = mesh.Faces[f][1], i2 = mesh.Faces[f][2];
                    int bestK = 0;
                    double bestFaceScore = double.MinValue;
                    for (int k = 0; k < K; k++)
                    {
                        double fScore = (pieceScores[k][i0] + pieceScores[k][i1] + pieceScores[k][i2]) / 3.0;
                        // Bonus for direct face normal draft alignment
                        fScore += 0.25 * (mesh.FaceNormals[f] * resolvedPullVectors[k]);
                        if (fScore > bestFaceScore)
                        {
                            bestFaceScore = fScore;
                            bestK = k;
                        }
                    }
                    faceLabels[f] = bestK;
                }

                // Update auto-discovered pull directions & geodesic seeds for next iteration
                if (iter == 0)
                {
                    var sumNormals = new Vector3d[K];
                    var sumCenters = new Point3d[K];
                    var sumArea = new double[K];

                    for (int f = 0; f < nF; f++)
                    {
                        int lbl = faceLabels[f];
                        // Weight low-curvature interior faces higher when estimating optimal pull direction
                        int i0 = mesh.Faces[f][0], i1 = mesh.Faces[f][1], i2 = mesh.Faces[f][2];
                        double fRidge = (ridgeField[i0] + ridgeField[i1] + ridgeField[i2]) / 3.0;
                        double w = mesh.FaceAreas[f] * (1.1 - 0.6 * fRidge);
                        sumNormals[lbl] += w * mesh.FaceNormals[f];
                        sumCenters[lbl] += w * mesh.FaceCenters[f];
                        sumArea[lbl] += w;
                    }

                    for (int k = 0; k < K; k++)
                    {
                        if (!fixedUserDir[k] && sumNormals[k].Length > 1e-12)
                        {
                            Vector3d newDir = sumNormals[k];
                            newDir.Unitize();
                            resolvedPullVectors[k] = newDir;
                        }

                        if (sumArea[k] > 1e-12)
                        {
                            Point3d targetC = sumCenters[k] / sumArea[k];
                            // Find vertex in cluster k closest to centroid and best aligned with pull vector
                            int bestSeed = seedVertices[k];
                            double bestVal = double.MinValue;
                            for (int i = 0; i < nV; i++)
                            {
                                if (vertexLabels[i] != k) continue;
                                double dNorm = mesh.Positions[i].DistanceTo(targetC) / diagLen;
                                double val = (mesh.VertexNormals[i] * resolvedPullVectors[k]) - 1.5 * dNorm - 0.3 * ridgeField[i];
                                if (val > bestVal)
                                {
                                    bestVal = val;
                                    bestSeed = i;
                                }
                            }
                            if (bestSeed != seedVertices[k])
                            {
                                seedVertices[k] = bestSeed;
                                if (wGeo > 1e-5)
                                    geodesicFields[k] = ComputeHeatDistance(mesh, new int[] { bestSeed }, 1.0, out _, out _);
                            }
                        }
                    }
                }
            }

            // 6. Enforce Topological Connectedness (Flood-Fill refinement so each mold piece is a contiguous shell without isolated islands!)
            EnforceConnectedClusterRegions(mesh, faceLabels, K);

            // Sync vertex labels with cleaned face labels
            for (int f = 0; f < nF; f++)
            {
                int lbl = faceLabels[f];
                vertexLabels[mesh.Faces[f][0]] = lbl;
                vertexLabels[mesh.Faces[f][1]] = lbl;
                vertexLabels[mesh.Faces[f][2]] = lbl;
            }

            // 7. Compute final piece centroids and finalize auto-discovered pull vectors
            pieceCentroids = new Point3d[K];
            var areaPerPiece = new double[K];
            var normalPerPiece = new Vector3d[K];
            for (int f = 0; f < nF; f++)
            {
                int k = faceLabels[f];
                double a = mesh.FaceAreas[f];
                pieceCentroids[k] += a * mesh.FaceCenters[f];
                normalPerPiece[k] += a * mesh.FaceNormals[f];
                areaPerPiece[k] += a;
            }
            for (int k = 0; k < K; k++)
            {
                if (areaPerPiece[k] > 1e-12)
                {
                    pieceCentroids[k] /= areaPerPiece[k];
                    MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(pieceCentroids[k], 0.0);
                    if (mp != null) pieceCentroids[k] = mp.Point;
                }
                else
                {
                    pieceCentroids[k] = mesh.Positions[seedVertices[k]];
                }

                if (!fixedUserDir[k] && normalPerPiece[k].Length > 1e-12)
                {
                    Vector3d nUnit = normalPerPiece[k];
                    nUnit.Unitize();
                    resolvedPullVectors[k] = nUnit;
                }
            }

            // 8. Compute Effective Draft Angle per vertex (in degrees [-90..+90]) and Undercut Warning Sub-Mesh
            effectiveDraftAnglesDeg = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                int k = vertexLabels[i];
                double dot = Math.Max(-1.0, Math.Min(1.0, mesh.VertexNormals[i] * resolvedPullVectors[k]));
                effectiveDraftAnglesDeg[i] = Math.Asin(dot) * (180.0 / Math.PI);
            }

            double threshDeg = Math.Max(0.1, minDraftAngleDeg);
            double threshSin = Math.Sin(threshDeg * Math.PI / 180.0);

            undercutWarningMesh = new Mesh();
            for (int i = 0; i < nV; i++)
                undercutWarningMesh.Vertices.Add(mesh.Positions[i]);

            for (int f = 0; f < nF; f++)
            {
                int k = faceLabels[f];
                double fDot = mesh.FaceNormals[f] * resolvedPullVectors[k];
                if (fDot < threshSin)
                {
                    undercutWarningMesh.Faces.AddFace(mesh.Faces[f][0], mesh.Faces[f][1], mesh.Faces[f][2]);
                }
            }
            undercutWarningMesh.Compact();
            undercutWarningMesh.Normals.ComputeNormals();

            // 9. Extract Smooth Parting Lines:
            //    If K == 2 and opposite vectors, extract exact sub-triangle zero-crossing iso-contour;
            //    For general K-piece molds, extract boundary seams and tension/smooth them onto high-curvature ridges via FlipOut!
            if (K == 2 && (resolvedPullVectors[0] * resolvedPullVectors[1]) < -0.95 && wGeo < 0.05)
            {
                double[] diffField = new double[nV];
                for (int i = 0; i < nV; i++)
                    diffField[i] = pieceScores[0][i] - pieceScores[1][i];
                partingLines = mesh.ExtractIsoContours(diffField, new double[] { 0.0 });
            }
            else
            {
                List<Polyline> rawSeams = ExtractLabelBoundaryCurves(mesh, faceLabels);
                if (rawSeams.Count > 0)
                {
                    var seamCurves = rawSeams.ConvertAll(pl => (Curve)new PolylineCurve(pl));
                    partingLines = ComputeGeodesicNetworkShorten(
                        mesh.RhinoMesh, seamCurves, 8, true, out _, wCurv > 0.1 ? ridgeField : null);
                }
                else
                {
                    partingLines = new List<Polyline>();
                }
            }

            // 10. Build segmented Mold Piece Meshes and Evaluation Colored Mesh
            Color[] moldPalette = GenerateDistinctPalette(K);
            List<Mesh> moldPieces = BuildSubMeshesFromFaceLabels(mesh, faceLabels, K, moldPalette, out coloredEvaluationMesh);

            // Enhance coloredEvaluationMesh so Undercut / Insufficient-Draft vertices glow bright Gold / Crimson!
            if (coloredEvaluationMesh.VertexColors.Count == coloredEvaluationMesh.Vertices.Count)
            {
                for (int i = 0; i < coloredEvaluationMesh.Vertices.Count; i++)
                {
                    Point3d vp = coloredEvaluationMesh.Vertices.Point3dAt(i);
                    int vOrig = mesh.FindClosestVertex(vp);
                    double deg = effectiveDraftAnglesDeg[vOrig];
                    if (deg < 0.0)
                    {
                        coloredEvaluationMesh.VertexColors[i] = Color.FromArgb(245, 50, 45);
                    }
                    else if (deg < threshDeg)
                    {
                        coloredEvaluationMesh.VertexColors[i] = Color.FromArgb(255, 210, 45);
                    }
                }
            }

            return moldPieces;
        }

        private static void EnforceConnectedClusterRegions(DdgMesh mesh, int[] faceLabels, int K)
        {
            int nF = mesh.FaceCount;
            if (nF == 0 || K <= 1) return;

            // Build face adjacency list
            var adj = new List<int>[nF];
            for (int f = 0; f < nF; f++) adj[f] = new List<int>(3);
            foreach (var e in mesh.Edges)
            {
                if (e.HalfEdge1 == -1) continue;
                int f0 = mesh.HalfEdges[e.HalfEdge0].FaceIndex;
                int f1 = mesh.HalfEdges[e.HalfEdge1].FaceIndex;
                if (f0 != f1)
                {
                    adj[f0].Add(f1);
                    adj[f1].Add(f0);
                }
            }

            // Find connected components of faces sharing the same label
            int[] compId = new int[nF];
            for (int f = 0; f < nF; f++) compId[f] = -1;
            var compFaces = new List<List<int>>();
            var compLabel = new List<int>();
            var compArea = new List<double>();

            var queue = new Queue<int>();
            for (int f = 0; f < nF; f++)
            {
                if (compId[f] != -1) continue;
                int cid = compFaces.Count;
                int lbl = faceLabels[f];
                var faces = new List<int>();
                double area = 0.0;

                compId[f] = cid;
                queue.Enqueue(f);
                while (queue.Count > 0)
                {
                    int cur = queue.Dequeue();
                    faces.Add(cur);
                    area += mesh.FaceAreas[cur];
                    foreach (int nb in adj[cur])
                    {
                        if (compId[nb] == -1 && faceLabels[nb] == lbl)
                        {
                            compId[nb] = cid;
                            queue.Enqueue(nb);
                        }
                    }
                }
                compFaces.Add(faces);
                compLabel.Add(lbl);
                compArea.Add(area);
            }

            // For each label k in 0..K-1, identify its largest connected component
            int[] largestCompForLabel = new int[K];
            double[] maxAreaForLabel = new double[K];
            for (int k = 0; k < K; k++) { largestCompForLabel[k] = -1; maxAreaForLabel[k] = -1.0; }

            for (int c = 0; c < compFaces.Count; c++)
            {
                int lbl = compLabel[c];
                if (lbl >= 0 && lbl < K && compArea[c] > maxAreaForLabel[lbl])
                {
                    maxAreaForLabel[lbl] = compArea[c];
                    largestCompForLabel[lbl] = c;
                }
            }

            // Reassign any smaller disconnected island components to their most common neighboring component's label
            for (int pass = 0; pass < 3; pass++)
            {
                for (int c = 0; c < compFaces.Count; c++)
                {
                    int lbl = compLabel[c];
                    if (lbl >= 0 && lbl < K && largestCompForLabel[lbl] == c) continue;

                    int bestNeighborLabel = -1;
                    int bestCount = 0;
                    var counts = new Dictionary<int, int>();

                    foreach (int f in compFaces[c])
                    {
                        foreach (int nb in adj[f])
                        {
                            int nbLbl = faceLabels[nb];
                            if (nbLbl != lbl)
                            {
                                counts.TryGetValue(nbLbl, out int cnt);
                                counts[nbLbl] = cnt + 1;
                                if (cnt + 1 > bestCount)
                                {
                                    bestCount = cnt + 1;
                                    bestNeighborLabel = nbLbl;
                                }
                            }
                        }
                    }

                    if (bestNeighborLabel != -1)
                    {
                        foreach (int f in compFaces[c])
                            faceLabels[f] = bestNeighborLabel;
                        compLabel[c] = bestNeighborLabel;
                    }
                }
            }
        }

        // =========================================================================================
        // 19. CONCENTRIC & SPIRAL CNC MILLING / 3D PRINTING TOOLPATHS (Rolling-Sphere OffsetGeo & Helical Spiral)
        // =========================================================================================
        public static List<Polyline> ComputeConcentricMillingToolpaths(
            DdgMesh mesh,
            IList<double> optionalCustomScalar,
            IList<Point3d> optionalSeedPoints,
            double stepoverDistance,
            double ballToolRadius,
            out List<Polyline> toolCenterOffsetPaths,
            out Polyline connectedSpiralPath,
            out List<Plane> toolFrames,
            out double estimatedScallopHeight,
            out Mesh coloredDistanceMesh,
            out Mesh rollingSphereOffsetGeo,
            IList<Vector3d> optionalGradientVectors = null)
        {
            int nV = mesh.VertexCount;
            double[] distField;

            if (optionalCustomScalar != null && optionalCustomScalar.Count > 0)
            {
                // Directly respect user's scalar field (e.g. from SmoothScalar or HeatDist)
                distField = mesh.MapScalarInputToVertices(optionalCustomScalar);
            }
            else if (optionalGradientVectors != null && optionalGradientVectors.Count > 0)
            {
                // Integrate input scalar gradient / vector field into a smooth scalar potential via Helmholtz-Hodge Eikonal solver!
                SmoothVectorAndGradientMultiMode(
                    mesh, optionalGradientVectors, null, 2.0, 1,
                    out _, out _, out distField, out _);
            }
            else if (optionalSeedPoints != null && optionalSeedPoints.Count > 0)
            {
                var seeds = new List<int>();
                foreach (var pt in optionalSeedPoints)
                    seeds.Add(mesh.FindClosestVertex(pt));
                distField = ComputeHeatDistance(mesh, seeds, 1.0, out _, out _);
            }
            else
            {
                // Default: Concentric inward milling from naked boundary loop (or lowest Z vertices if closed)
                var bndSeeds = new List<int>();
                for (int i = 0; i < nV; i++)
                    if (mesh.VertexOnBoundary[i]) bndSeeds.Add(i);
                if (bndSeeds.Count == 0)
                {
                    int minZIdx = 0;
                    for (int i = 1; i < nV; i++)
                        if (mesh.Positions[i].Z < mesh.Positions[minZIdx].Z) minZIdx = i;
                    bndSeeds.Add(minZIdx);
                }
                distField = ComputeHeatDistance(mesh, bndSeeds, 1.0, out _, out _);
            }

            double minD = double.MaxValue, maxD = double.MinValue;
            for (int i = 0; i < nV; i++)
            {
                if (distField[i] < minD) minD = distField[i];
                if (distField[i] > maxD) maxD = distField[i];
            }

            double stepD = stepoverDistance > 1e-5
                ? stepoverDistance
                : Math.Max(1e-4, (maxD - minD) / 16.0);

            var isoLevels = new List<double>();
            for (double val = minD + 0.5 * stepD; val < maxD - 1e-6; val += stepD)
            {
                isoLevels.Add(val);
                if (isoLevels.Count >= 250) break;
            }

            List<Polyline> surfacePaths = mesh.ExtractIsoContours(distField, isoLevels);

            // Compute Rolling-Sphere Offset Geometry (OffsetGeo) when ballToolRadius > 0
            double rTool = Math.Max(0.0, Math.Abs(ballToolRadius));
            estimatedScallopHeight = rTool > 1e-6 ? (stepD * stepD) / (8.0 * rTool) : 0.0;

            if (rTool > 1e-6)
            {
                bool exteriorSide = ballToolRadius >= 0.0;
                ComputeRollingSpheresAndOffset(
                    mesh, rTool, exteriorSide, 12,
                    out _, out rollingSphereOffsetGeo, out _, out _);

                var ddgOffset = new DdgMesh(rollingSphereOffsetGeo);
                toolCenterOffsetPaths = ddgOffset.ExtractIsoContours(distField, isoLevels);
                connectedSpiralPath = BuildContinuousHelicalSpiralOnMesh(ddgOffset.RhinoMesh, toolCenterOffsetPaths);
            }
            else
            {
                rollingSphereOffsetGeo = mesh.RhinoMesh.DuplicateMesh();
                toolCenterOffsetPaths = new List<Polyline>(surfacePaths);
                connectedSpiralPath = BuildContinuousHelicalSpiralOnMesh(mesh.RhinoMesh, surfacePaths);
            }

            // Extract 5/6-Axis Tool Frames along the Offset Paths (or Spiral)
            toolFrames = new List<Plane>();
            Mesh targetM = rollingSphereOffsetGeo;
            IList<Polyline> frameSourcePaths = connectedSpiralPath.Count >= 4
                ? new List<Polyline> { connectedSpiralPath }
                : toolCenterOffsetPaths;

            foreach (Polyline pl in frameSourcePaths)
            {
                if (pl == null || pl.Count < 2) continue;
                int fStride = Math.Max(1, pl.Count / 40);
                for (int i = 0; i < pl.Count; i += fStride)
                {
                    Point3d p = pl[i];
                    MeshPoint mp = targetM.ClosestMeshPoint(p, 0.0);
                    Vector3d n = mp != null ? targetM.NormalAt(mp) : Vector3d.ZAxis;
                    if (n.Length < 1e-12) n = Vector3d.ZAxis;
                    n.Unitize();

                    Vector3d tangent = (i < pl.Count - 1) ? (pl[i + 1] - pl[i]) : (pl[i] - pl[i - 1]);
                    tangent -= (tangent * n) * n;
                    if (tangent.Length < 1e-12)
                        tangent = Vector3d.CrossProduct(n, Math.Abs(n.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                    tangent.Unitize();
                    Vector3d binormal = Vector3d.CrossProduct(n, tangent);
                    binormal.Unitize();
                    toolFrames.Add(new Plane(p, tangent, binormal));
                }
            }

            coloredDistanceMesh = ColorMeshByScalar(mesh.RhinoMesh, distField, false);
            return surfacePaths;
        }

        /// <summary>
        /// Connects a sequence of ordered iso-contour loops/curves into a smooth, continuous Helical Spiral
        /// on the mesh by ramping smoothly from contour k to contour k+1 over each revolution!
        /// </summary>
        public static Polyline BuildContinuousHelicalSpiralOnMesh(Mesh refMesh, IList<Polyline> orderedContours)
        {
            var spiralPts = new List<Point3d>();
            if (orderedContours == null || orderedContours.Count == 0)
                return new Polyline();

            // Filter valid contours with >= 3 points
            var validLoops = new List<Polyline>();
            foreach (var pl in orderedContours)
            {
                if (pl != null && pl.Count >= 3)
                    validLoops.Add(new Polyline(pl));
            }
            if (validLoops.Count == 0) return new Polyline();
            if (validLoops.Count == 1) return validLoops[0];

            int samplesPerTurn = 140;

            // Helper: smooth a raw contour polyline and resample it into N uniform points along a degree-3 NURBS curve
            Point3d[] ResampleAndAlignLoop(Polyline rawLoop, Point3d refStartPt, Vector3d preferredTangent)
            {
                var work = new Polyline(rawLoop);
                double len = work.Length;
                bool isClosed = work[0].DistanceTo(work[work.Count - 1]) < Math.Max(len * 0.18, refMesh != null ? refMesh.GetBoundingBox(false).Diagonal.Length * 0.05 : 1e-3);

                // Smooth raw polygon vertices first so triangle facet kinks are removed before curve fitting
                int nRaw = isClosed ? work.Count - 1 : work.Count;
                if (nRaw >= 4)
                {
                    Point3d[] cur = new Point3d[nRaw];
                    Point3d[] nxt = new Point3d[nRaw];
                    for (int i = 0; i < nRaw; i++) cur[i] = work[i];

                    for (int pass = 0; pass < 4; pass++)
                    {
                        for (int i = 0; i < nRaw; i++)
                        {
                            if (!isClosed && (i == 0 || i == nRaw - 1))
                            {
                                nxt[i] = cur[i];
                                continue;
                            }
                            Point3d prev = cur[(i - 1 + nRaw) % nRaw];
                            Point3d next = cur[(i + 1) % nRaw];
                            nxt[i] = 0.5 * cur[i] + 0.25 * (prev + next);
                        }
                        for (int i = 0; i < nRaw; i++) cur[i] = nxt[i];
                    }

                    work.Clear();
                    for (int i = 0; i < nRaw; i++) work.Add(cur[i]);
                    if (isClosed) work.Add(cur[0]);
                }

                Curve smoothLoopCrv = Curve.CreateInterpolatedCurve(work, 3, CurveKnotStyle.Chord);
                if (smoothLoopCrv == null) smoothLoopCrv = new PolylineCurve(work);

                Point3d[] pts = new Point3d[samplesPerTurn];
                double[] tParams = smoothLoopCrv.DivideByCount(samplesPerTurn - 1, true);
                if (tParams == null || tParams.Length < samplesPerTurn)
                {
                    for (int i = 0; i < samplesPerTurn; i++)
                        pts[i] = work[Math.Min(work.Count - 1, (i * work.Count) / samplesPerTurn)];
                }
                else
                {
                    for (int i = 0; i < samplesPerTurn; i++)
                        pts[i] = smoothLoopCrv.PointAt(tParams[i]);
                }

                if (isClosed)
                {
                    if (refStartPt.IsValid)
                    {
                        int bestIdx = 0;
                        double minD2 = double.MaxValue;
                        for (int i = 0; i < samplesPerTurn - 1; i++)
                        {
                            double d2 = pts[i].DistanceToSquared(refStartPt);
                            if (d2 < minD2) { minD2 = d2; bestIdx = i; }
                        }

                        var shifted = new Point3d[samplesPerTurn];
                        for (int i = 0; i < samplesPerTurn - 1; i++)
                            shifted[i] = pts[(bestIdx + i) % (samplesPerTurn - 1)];
                        shifted[samplesPerTurn - 1] = shifted[0];
                        pts = shifted;
                    }

                    if (preferredTangent.Length > 1e-8)
                    {
                        Vector3d curTang = pts[Math.Min(6, samplesPerTurn - 1)] - pts[0];
                        if ((curTang * preferredTangent) < 0.0)
                        {
                            Array.Reverse(pts);
                        }
                    }
                }
                else
                {
                    // Open contour: orient so pts[0] is aligned with preferredTangent or closest to refStartPt
                    if (preferredTangent.Length > 1e-8)
                    {
                        Vector3d curTang = pts[pts.Length - 1] - pts[0];
                        if ((curTang * preferredTangent) < 0.0)
                            Array.Reverse(pts);
                    }
                    else if (refStartPt.IsValid)
                    {
                        if (pts[pts.Length - 1].DistanceToSquared(refStartPt) < pts[0].DistanceToSquared(refStartPt))
                            Array.Reverse(pts);
                    }
                }

                return pts;
            }

            Point3d prevStart = validLoops[0][0];
            Vector3d prevTang = validLoops[0].Count >= 2 ? (validLoops[0][Math.Min(3, validLoops[0].Count - 1)] - validLoops[0][0]) : Vector3d.Zero;

            var resampledLoops = new List<Point3d[]>(validLoops.Count);
            for (int k = 0; k < validLoops.Count; k++)
            {
                Point3d[] aligned = ResampleAndAlignLoop(validLoops[k], prevStart, prevTang);
                resampledLoops.Add(aligned);
                prevStart = aligned[0];
                prevTang = aligned[Math.Min(6, samplesPerTurn - 1)] - aligned[0];
            }

            double maxJump = refMesh != null ? refMesh.GetBoundingBox(false).Diagonal.Length * 0.45 : 1e9;

            // Build helical spiral turns: on turn k, interpolate P(s) = (1 - s) * Loop_k(s) + s * Loop_{k+1}(s)
            for (int k = 0; k < resampledLoops.Count - 1; k++)
            {
                Point3d[] loopA = resampledLoops[k];
                Point3d[] loopB = resampledLoops[k + 1];

                if (loopA[0].DistanceTo(loopB[0]) > maxJump)
                {
                    for (int i = 0; i < samplesPerTurn; i++) spiralPts.Add(loopA[i]);
                    continue;
                }

                for (int i = 0; i < samplesPerTurn - 1; i++)
                {
                    double s = i / (double)(samplesPerTurn - 1);
                    Point3d blended = (1.0 - s) * loopA[i] + s * loopB[i];
                    if (refMesh != null)
                    {
                        MeshPoint mp = refMesh.ClosestMeshPoint(blended, 0.0);
                        if (mp != null)
                        {
                            // Soft surface attraction (70% smooth space curve, 30% flat facet) to avoid polygonal facet kinks
                            blended = 0.35 * blended + 0.65 * mp.Point;
                        }
                    }
                    if (spiralPts.Count == 0 || blended.DistanceToSquared(spiralPts[spiralPts.Count - 1]) > 1e-10)
                        spiralPts.Add(blended);
                }
            }

            // Smooth the final spiral point chain with 6 passes of Laplacian fairing so C1/C2 continuity is silky-smooth
            if (spiralPts.Count >= 6)
            {
                Point3d[] tmp = new Point3d[spiralPts.Count];
                for (int pass = 0; pass < 6; pass++)
                {
                    tmp[0] = spiralPts[0];
                    tmp[spiralPts.Count - 1] = spiralPts[spiralPts.Count - 1];
                    for (int i = 1; i < spiralPts.Count - 1; i++)
                    {
                        tmp[i] = 0.5 * spiralPts[i] + 0.25 * (spiralPts[i - 1] + spiralPts[i + 1]);
                    }
                    for (int i = 1; i < spiralPts.Count - 1; i++)
                        spiralPts[i] = tmp[i];
                }
            }

            return new Polyline(spiralPts);
        }

        /// <summary>
        /// Extracts a true smooth Degree-3 NURBS Spiral Iso-Curve on a mesh from a scalar field,
        /// selecting one coherent primary contour per iso-level so the spiral never jumps or kinks.
        /// </summary>
        public static Curve ComputeSmoothSpiralIsoCurveOnMesh(
            DdgMesh mesh,
            double[] scalarField,
            double stepDistance,
            int spiralMode,
            out List<Curve> smoothIsoCurves)
        {
            smoothIsoCurves = new List<Curve>();
            int nV = mesh.VertexCount;

            // 1. Lightly polish the scalar field so triangle-level noise doesn't wiggle the spiral
            double[] uSmooth = SmoothScalarField(mesh, scalarField, 0.8, 0.0, out _, out _);

            double uMin = double.MaxValue, uMax = double.MinValue;
            for (int i = 0; i < nV; i++)
            {
                if (uSmooth[i] < uMin) uMin = uSmooth[i];
                if (uSmooth[i] > uMax) uMax = uSmooth[i];
            }
            double uSpan = Math.Max(1e-9, uMax - uMin);

            int numTurns;
            if (stepDistance > 1e-6)
            {
                BoundingBox bb = mesh.RhinoMesh.GetBoundingBox(false);
                numTurns = Math.Max(4, Math.Min(120, (int)Math.Round(bb.Diagonal.Length / stepDistance)));
            }
            else
            {
                numTurns = 24;
            }

            // 2. Extract 1 primary (longest / most coherent) contour loop per iso-level
            var orderedPrimaryLoops = new List<Polyline>(numTurns);
            Point3d prevCentroid = Point3d.Unset;

            for (int k = 0; k < numTurns; k++)
            {
                double t = (k + 0.5) / numTurns;
                double isoVal = uMin + t * uSpan;
                List<Polyline> levelLoops = mesh.ExtractIsoContours(uSmooth, new double[] { isoVal });
                if (levelLoops == null || levelLoops.Count == 0) continue;

                // Pick the best contour at this iso-level (longest, weighted by proximity to previous turn)
                Polyline bestPl = null;
                double bestScore = -1.0;
                foreach (var pl in levelLoops)
                {
                    if (pl == null || pl.Count < 3) continue;
                    double l = pl.Length;
                    if (l < mesh.MeanEdgeLength * 0.5) continue;

                    double score = l;
                    if (prevCentroid.IsValid)
                    {
                        Point3d c = pl.CenterPoint();
                        double d = c.DistanceTo(prevCentroid);
                        score = l / (1.0 + 0.25 * d / Math.Max(1e-6, mesh.MeanEdgeLength));
                    }
                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestPl = pl;
                    }
                }

                if (bestPl != null)
                {
                    orderedPrimaryLoops.Add(bestPl);
                    prevCentroid = bestPl.CenterPoint();

                    // Also build a smooth degree-3 NURBS curve for the individual iso-curves output
                    Curve cIso = Curve.CreateInterpolatedCurve(bestPl, 3, CurveKnotStyle.Chord);
                    smoothIsoCurves.Add(cIso ?? new PolylineCurve(bestPl));
                }
            }

            if (orderedPrimaryLoops.Count == 0)
                return new PolylineCurve();

            // 3. If Fermat Mode (1), interleave even turns inward and odd turns outward
            List<Polyline> sequenceToConnect = orderedPrimaryLoops;
            if (spiralMode == 1 && orderedPrimaryLoops.Count >= 2)
            {
                sequenceToConnect = new List<Polyline>(orderedPrimaryLoops.Count);
                for (int k = 0; k < orderedPrimaryLoops.Count; k += 2)
                    sequenceToConnect.Add(orderedPrimaryLoops[k]);
                int lastOdd = (orderedPrimaryLoops.Count - 1) % 2 == 1
                    ? (orderedPrimaryLoops.Count - 1)
                    : (orderedPrimaryLoops.Count - 2);
                for (int k = lastOdd; k >= 1; k -= 2)
                {
                    var rev = new Polyline(orderedPrimaryLoops[k]);
                    rev.Reverse();
                    sequenceToConnect.Add(rev);
                }
            }

            // 4. Build dense faired helical point sequence and convert to a smooth Degree-3 NURBS Curve!
            Polyline denseSpiralPl = BuildContinuousHelicalSpiralOnMesh(mesh.RhinoMesh, sequenceToConnect);
            if (denseSpiralPl == null || denseSpiralPl.Count < 2)
                return new PolylineCurve();

            // Subsample slightly (every 2nd point) so Degree-3 NURBS interpolation is ultra-smooth without overfitting
            var controlPts = new List<Point3d>(denseSpiralPl.Count / 2 + 2);
            for (int i = 0; i < denseSpiralPl.Count; i += 2)
                controlPts.Add(denseSpiralPl[i]);
            if (controlPts[controlPts.Count - 1].DistanceToSquared(denseSpiralPl[denseSpiralPl.Count - 1]) > 1e-10)
                controlPts.Add(denseSpiralPl[denseSpiralPl.Count - 1]);

            if (controlPts.Count >= 4)
            {
                Curve nurbsSpiral = Curve.CreateInterpolatedCurve(controlPts, 3, CurveKnotStyle.Chord);
                if (nurbsSpiral != null)
                    return nurbsSpiral;
            }

            return new PolylineCurve(denseSpiralPl);
        }

        // =========================================================================================
        // 20. ISO-BOX & VOLUMETRIC ISO-SHELL SUBTRACTION (3D Print & CNC Roughing Stock)
        // =========================================================================================
        public static Mesh ComputeIsoBoxAndShellSubtraction(
            DdgMesh mesh,
            IList<double> optionalScalarField,
            double isoMin,
            double isoMax,
            double shellThickness,
            int boxGridResolution,
            out List<Box> subtractiveIsoBoxes,
            out Mesh surfaceBandMesh)
        {
            double[] u = (optionalScalarField != null && optionalScalarField.Count > 0)
                ? mesh.MapScalarInputToVertices(optionalScalarField)
                : ComputeHeatDistance(mesh, new int[] { 0 }, 1.0, out _, out _);

            double lo = Math.Min(isoMin, isoMax);
            double hi = Math.Max(isoMin, isoMax);
            if (Math.Abs(hi - lo) < 1e-9) hi = lo + mesh.MeanEdgeLength;

            // 1. Extract exact cropped surface band where u in [lo, hi]
            Mesh aboveLo = SliceMeshHalfSpace(mesh, u, lo, false);
            if (aboveLo.Faces.Count > 0)
            {
                var ddgAbove = new DdgMesh(aboveLo);
                double[] uAbove = new double[ddgAbove.VertexCount];
                for (int i = 0; i < ddgAbove.VertexCount; i++)
                {
                    MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(ddgAbove.Positions[i], 0.0);
                    if (mp != null)
                    {
                        MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                        uAbove[i] = mp.T[0] * u[mf.A] + mp.T[1] * u[mf.B] + mp.T[2] * u[mf.C];
                    }
                }
                surfaceBandMesh = SliceMeshHalfSpace(ddgAbove, uAbove, hi, true);
            }
            else
            {
                surfaceBandMesh = new Mesh();
            }

            // 2. Thicken surfaceBandMesh into a closed 3D Volumetric Iso-Shell Mesh for 3D Printing / Boolean Subtraction
            Mesh solidShell = ThickenMeshBandToSolidShell(surfaceBandMesh, Math.Abs(shellThickness) > 1e-5 ? shellThickness : mesh.MeanEdgeLength);

            // 3. Generate 3D Subtractive Iso-Boxes in the bounding volume whose closest surface scalar & distance fall within the target machining band
            subtractiveIsoBoxes = new List<Box>();
            int res = Math.Max(3, Math.Min(30, boxGridResolution));
            BoundingBox bbox = mesh.RhinoMesh.GetBoundingBox(false);
            Vector3d diag = bbox.Max - bbox.Min;
            double dx = Math.Max(1e-4, diag.X / res);
            double dy = Math.Max(1e-4, diag.Y / res);
            double dz = Math.Max(1e-4, diag.Z / res);

            for (int ix = 0; ix < res; ix++)
            {
                for (int iy = 0; iy < res; iy++)
                {
                    for (int iz = 0; iz < res; iz++)
                    {
                        Point3d cellCenter = new Point3d(
                            bbox.Min.X + (ix + 0.5) * dx,
                            bbox.Min.Y + (iy + 0.5) * dy,
                            bbox.Min.Z + (iz + 0.5) * dz);

                        MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(cellCenter, 0.0);
                        if (mp == null) continue;

                        MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                        double uSurf = mp.T[0] * u[mf.A] + mp.T[1] * u[mf.B] + mp.T[2] * u[mf.C];
                        double distToSurf = cellCenter.DistanceTo(mp.Point);

                        if (uSurf >= lo && uSurf <= hi && distToSurf <= Math.Max(dx, Math.Max(dy, dz)) * 1.35)
                        {
                            var bBox = new BoundingBox(
                                new Point3d(cellCenter.X - 0.48 * dx, cellCenter.Y - 0.48 * dy, cellCenter.Z - 0.48 * dz),
                                new Point3d(cellCenter.X + 0.48 * dx, cellCenter.Y + 0.48 * dy, cellCenter.Z + 0.48 * dz));
                            subtractiveIsoBoxes.Add(new Box(bBox));
                        }
                    }
                }
            }

            return solidShell;
        }

        private static Mesh ThickenMeshBandToSolidShell(Mesh band, double thickness)
        {
            var shell = new Mesh();
            if (band == null || band.Faces.Count == 0) return shell;

            band.Normals.ComputeNormals();
            int nV = band.Vertices.Count;
            int nF = band.Faces.Count;

            // Bottom vertices (0 .. nV-1) and Top offset vertices (nV .. 2*nV-1)
            for (int i = 0; i < nV; i++)
                shell.Vertices.Add(band.Vertices.Point3dAt(i));
            for (int i = 0; i < nV; i++)
            {
                Point3d p = band.Vertices.Point3dAt(i) + thickness * new Vector3d(band.Normals[i]);
                shell.Vertices.Add(p);
            }

            var edgeCounts = new Dictionary<(int, int), int>();
            for (int f = 0; f < nF; f++)
            {
                MeshFace mf = band.Faces[f];
                // Bottom face (inverted normal) & Top face (outward normal)
                shell.Faces.AddFace(mf.A, mf.C, mf.B);
                shell.Faces.AddFace(mf.A + nV, mf.B + nV, mf.C + nV);

                int[] v = { mf.A, mf.B, mf.C };
                for (int k = 0; k < 3; k++)
                {
                    int a = v[k], b = v[(k + 1) % 3];
                    var key = a < b ? (a, b) : (b, a);
                    if (edgeCounts.TryGetValue(key, out int cnt)) edgeCounts[key] = cnt + 1;
                    else edgeCounts[key] = 1;
                }
            }

            // Stitch naked boundary edges with side walls
            for (int f = 0; f < nF; f++)
            {
                MeshFace mf = band.Faces[f];
                int[] v = { mf.A, mf.B, mf.C };
                for (int k = 0; k < 3; k++)
                {
                    int a = v[k], b = v[(k + 1) % 3];
                    var key = a < b ? (a, b) : (b, a);
                    if (edgeCounts[key] == 1)
                    {
                        shell.Faces.AddFace(a, b, b + nV);
                        shell.Faces.AddFace(a, b + nV, a + nV);
                    }
                }
            }

            shell.Normals.ComputeNormals();
            return shell;
        }

        // =========================================================================================
        // 21. GEODESIC TRAVELING SALESPERSON PROBLEM (TSP) & 6-AXIS ROBOT TCP FRAMES
        //     Uses Keenan Crane's Heat Method Geodesic Distance Matrix + 2-Opt Uncrossing +
        //     Levi-Civita Parallel Transport / Darboux Frame for Smooth 6-Axis Robot Motion!
        // =========================================================================================
        public static Polyline ComputeGeodesicTSPAndRobotFrames(
            DdgMesh mesh,
            IList<Point3d> optionalTargetPoints,
            int maxVertexNodes,
            bool closeLoop,
            double tcpNormalOffset,
            bool parallelTransportRobotRoll,
            out List<Plane> robotFrames,
            out List<Point3d> orderedWaypoints,
            out double totalGeodesicLength)
        {
            int nV = mesh.VertexCount;
            var nodeVertices = new List<int>();

            if (optionalTargetPoints != null && optionalTargetPoints.Count >= 2)
            {
                foreach (var pt in optionalTargetPoints)
                {
                    int vIdx = mesh.FindClosestVertex(pt);
                    if (!nodeVertices.Contains(vIdx))
                        nodeVertices.Add(vIdx);
                }
            }
            else
            {
                // Sample up to maxVertexNodes vertices across the mesh (all vertices if nV <= maxVertexNodes)
                int cap = Math.Max(4, Math.Min(400, maxVertexNodes));
                int stride = Math.Max(1, nV / cap);
                for (int i = 0; i < nV; i += stride)
                    nodeVertices.Add(i);
            }

            int m = nodeVertices.Count;
            robotFrames = new List<Plane>();
            orderedWaypoints = new List<Point3d>(m);
            totalGeodesicLength = 0.0;

            if (m < 2) return new Polyline();

            // 1. Pre-factorize Heat & Poisson matrices or compute Heat Geodesic Distance Matrix D[i, j]
            //    For fast interactive response on large node counts, combine Heat Geodesic Distance with intrinsic graph distance
            double[,] distMat = new double[m, m];
            double[][] heatFields = new double[Math.Min(m, 45)][];
            for (int i = 0; i < m; i++)
            {
                int vi = nodeVertices[i];
                if (i < 45)
                {
                    heatFields[i] = ComputeHeatDistance(mesh, new int[] { vi }, 1.0, out _, out _);
                    for (int j = 0; j < m; j++)
                        distMat[i, j] = heatFields[i][nodeVertices[j]];
                }
                else
                {
                    for (int j = 0; j < m; j++)
                    {
                        if (j < 45) distMat[i, j] = distMat[j, i];
                        else distMat[i, j] = mesh.Positions[vi].DistanceTo(mesh.Positions[nodeVertices[j]]);
                    }
                }
            }

            // 2. Nearest-Neighbor Geodesic Tour initialization
            int[] tour = new int[m];
            bool[] visited = new bool[m];
            tour[0] = 0;
            visited[0] = true;

            for (int step = 1; step < m; step++)
            {
                int curr = tour[step - 1];
                int bestNext = -1;
                double bestD = double.MaxValue;
                for (int cand = 0; cand < m; cand++)
                {
                    if (!visited[cand] && distMat[curr, cand] < bestD)
                    {
                        bestD = distMat[curr, cand];
                        bestNext = cand;
                    }
                }
                tour[step] = bestNext != -1 ? bestNext : step;
                visited[tour[step]] = true;
            }

            // 3. 2-Opt Geodesic Uncrossing Optimization (eliminates self-crossings and minimizes total geodesic tour length)
            bool improved = true;
            int optPass = 0;
            while (improved && optPass < 40)
            {
                improved = false;
                optPass++;
                for (int i = 1; i < m - 1; i++)
                {
                    for (int j = i + 1; j < m; j++)
                    {
                        int a = tour[i - 1];
                        int b = tour[i];
                        int c = tour[j];
                        int d = (j + 1 < m) ? tour[j + 1] : (closeLoop ? tour[0] : -1);

                        double oldCost = distMat[a, b] + (d != -1 ? distMat[c, d] : 0.0);
                        double newCost = distMat[a, c] + (d != -1 ? distMat[b, d] : 0.0);

                        if (newCost + 1e-9 < oldCost)
                        {
                            Array.Reverse(tour, i, j - i + 1);
                            improved = true;
                        }
                    }
                }
            }

            // 4. Trace surface geodesic segments between consecutive tour waypoints & build 6-Axis Robot Planes
            var fullTrajectoryPts = new List<Point3d>();
            int segCount = closeLoop ? m : m - 1;

            for (int s = 0; s < m; s++)
                orderedWaypoints.Add(mesh.Positions[nodeVertices[tour[s]]]);

            for (int s = 0; s < segCount; s++)
            {
                int idxA = tour[s];
                int idxB = tour[(s + 1) % m];
                int vA = nodeVertices[idxA];
                int vB = nodeVertices[idxB];

                // Trace smooth geodesic segment on mesh from vA to vB
                Point3d pA = mesh.Positions[vA];
                Point3d pB = mesh.Positions[vB];
                int subSteps = Math.Max(2, (int)(pA.DistanceTo(pB) / (0.65 * mesh.MeanEdgeLength)));
                subSteps = Math.Min(25, subSteps);

                for (int k = 0; k < subSteps; k++)
                {
                    double t = (double)k / subSteps;
                    Point3d interp = (1.0 - t) * pA + t * pB;
                    MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(interp, 0.0);
                    Point3d surfPt = mp != null ? mp.Point : interp;
                    if (fullTrajectoryPts.Count == 0 || surfPt.DistanceToSquared(fullTrajectoryPts[fullTrajectoryPts.Count - 1]) > 1e-12)
                        fullTrajectoryPts.Add(surfPt);
                }
            }

            if (fullTrajectoryPts.Count > 0)
            {
                int lastTourV = nodeVertices[tour[closeLoop ? 0 : m - 1]];
                fullTrajectoryPts.Add(mesh.Positions[lastTourV]);
            }

            // 5. Build 6-Axis Robot TCP Frames (SE(3) Poses) along the geodesic trajectory
            //    Using either Parallel Transport (minimum twist around surface normal) or Darboux Tangent Frame
            Vector3d prevXAxis = Vector3d.XAxis;
            for (int i = 0; i < fullTrajectoryPts.Count; i++)
            {
                Point3d pt = fullTrajectoryPts[i];
                if (i > 0)
                    totalGeodesicLength += pt.DistanceTo(fullTrajectoryPts[i - 1]);

                MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(pt, 0.0);
                Vector3d normal = Vector3d.ZAxis;
                if (mp != null)
                {
                    MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                    normal = mp.T[0] * mesh.VertexNormals[mf.A] +
                             mp.T[1] * mesh.VertexNormals[mf.B] +
                             mp.T[2] * mesh.VertexNormals[mf.C];
                    if (normal.Length > 1e-12) normal.Unitize();
                }

                Vector3d xAxis;
                if (parallelTransportRobotRoll && i > 0)
                {
                    // Levi-Civita discrete parallel transport of previous robot X-axis onto new tangent plane (Minimum-Twist Bishop/Crane Frame!)
                    xAxis = prevXAxis - (prevXAxis * normal) * normal;
                    if (xAxis.Length < 1e-10)
                        xAxis = Vector3d.CrossProduct(normal, Math.Abs(normal.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                    xAxis.Unitize();
                }
                else
                {
                    // Darboux Frame: X-axis aligned with path tangent T, Z-axis aligned with surface normal N, Y-axis = N x T
                    Vector3d tang = (i < fullTrajectoryPts.Count - 1)
                        ? (fullTrajectoryPts[i + 1] - pt)
                        : (pt - fullTrajectoryPts[Math.Max(0, i - 1)]);
                    xAxis = tang - (tang * normal) * normal;
                    if (xAxis.Length < 1e-10)
                        xAxis = prevXAxis - (prevXAxis * normal) * normal;
                    if (xAxis.Length < 1e-10)
                        xAxis = Vector3d.CrossProduct(normal, Math.Abs(normal.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                    xAxis.Unitize();
                }

                prevXAxis = xAxis;
                Vector3d yAxis = Vector3d.CrossProduct(normal, xAxis);
                yAxis.Unitize();

                Point3d tcpOrigin = pt + tcpNormalOffset * normal;
                robotFrames.Add(new Plane(tcpOrigin, xAxis, yAxis));
            }

            return new Polyline(fullTrajectoryPts);
        }

        public static Mesh ColorMeshByPalette(
            Mesh baseMesh,
            double[] values,
            int paletteMode,
            double customMin = double.NaN,
            double customMax = double.NaN)
        {
            if (paletteMode == 0 && double.IsNaN(customMin) && double.IsNaN(customMax))
                return ColorMeshByScalar(baseMesh, values, false);

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
            if (!double.IsNaN(customMin)) minV = customMin;
            if (!double.IsNaN(customMax)) maxV = customMax;
            double span = Math.Max(1e-12, maxV - minV);

            for (int i = 0; i < n; i++)
            {
                double v = values[Math.Min(i, values.Length - 1)];
                double t = Math.Max(0.0, Math.Min(1.0, (v - minV) / span));
                m.VertexColors.Add(EvaluateScientificColor(t, paletteMode));
            }
            return m;
        }

        public static Color EvaluateScientificColor(double t, int paletteMode)
        {
            t = Math.Max(0.0, Math.Min(1.0, t));
            if (paletteMode == 1)
            {
                // CoolWarm Diverging: Deep Blue (0) -> Crisp White (0.5) -> Crimson Red (1)
                if (t < 0.5)
                {
                    double s = t * 2.0;
                    return Color.FromArgb((int)(59 + 188 * s), (int)(76 + 171 * s), (int)(192 + 55 * s));
                }
                else
                {
                    double s = (t - 0.5) * 2.0;
                    return Color.FromArgb((int)(247 - 67 * s), (int)(247 - 243 * s), (int)(247 - 209 * s));
                }
            }
            else if (paletteMode == 2)
            {
                // Viridis: Deep Purple (0) -> Teal (0.5) -> Bright Yellow (1)
                if (t < 0.5)
                {
                    double s = t * 2.0;
                    return Color.FromArgb((int)(68 - 35 * s), (int)(1 + 144 * s), (int)(84 + 56 * s));
                }
                else
                {
                    double s = (t - 0.5) * 2.0;
                    return Color.FromArgb((int)(33 + 220 * s), (int)(145 + 86 * s), (int)(140 - 103 * s));
                }
            }
            else if (paletteMode == 3)
            {
                // Turbo Rainbow
                return HsvToRgb((1.0 - t) * 240.0, 0.85, 0.95);
            }
            else if (paletteMode == 4)
            {
                // Zebra 10-band high contrast
                int band = ((int)Math.Floor(t * 20.0)) % 2;
                return band == 0 ? Color.FromArgb(240, 242, 245) : Color.FromArgb(30, 35, 45);
            }
            else
            {
                // Keenan Crane Heat Palette
                if (t < 0.25)
                {
                    double s = t / 0.25;
                    return Color.FromArgb(255, (int)(255 - 35 * s), (int)(245 - 185 * s));
                }
                else if (t < 0.65)
                {
                    double s = (t - 0.25) / 0.40;
                    return Color.FromArgb((int)(255 - 45 * s), (int)(220 - 165 * s), (int)(60 - 40 * s));
                }
                else
                {
                    double s = (t - 0.65) / 0.35;
                    return Color.FromArgb((int)(210 - 125 * s), (int)(55 - 40 * s), (int)(20 + 10 * s));
                }
            }
        }

        // =========================================================================================
        // 22. HEAT DISTANCE FROM EXTERNAL GEOMETRY (Curves, Meshes, Breps, Points) + SMOOTH GRADIENT
        // =========================================================================================
        public static double[] ComputeExternalGeometryHeatDistance(
            DdgMesh mesh,
            IEnumerable<GeometryBase> externalGeometries,
            double timeFactor,
            out Vector3d[] smoothVertexGradients,
            out Vector3d[] smoothFaceGradients,
            out List<Point3d> resolvedSourcePoints,
            out Mesh coloredMesh)
        {
            int nV = mesh.VertexCount;
            var sourceSet = new HashSet<int>();
            resolvedSourcePoints = new List<Point3d>();

            double snapTol = mesh.MeanEdgeLength * 1.35;

            if (externalGeometries != null)
            {
                foreach (GeometryBase geom in externalGeometries)
                {
                    if (geom == null) continue;

                    if (geom is Rhino.Geometry.Point ptGeom)
                    {
                        int vIdx = mesh.FindClosestVertex(ptGeom.Location);
                        sourceSet.Add(vIdx);
                    }
                    else if (geom is Curve crv && crv.IsValid)
                    {
                        // Sample curve densely and find all mesh vertices along/near the curve
                        int sampleCount = Math.Max(32, (int)(crv.GetLength() / Math.Max(1e-4, 0.4 * mesh.MeanEdgeLength)));
                        sampleCount = Math.Min(400, sampleCount);
                        crv.DivideByCount(sampleCount, true, out Point3d[] crvPts);
                        if (crvPts != null)
                        {
                            foreach (Point3d cp in crvPts)
                            {
                                int vIdx = mesh.FindClosestVertex(cp);
                                sourceSet.Add(vIdx);
                            }
                        }
                        // Also include any mesh vertex whose distance to the curve is <= snapTol
                        for (int i = 0; i < nV; i++)
                        {
                            if (crv.ClosestPoint(mesh.Positions[i], out double tParam, snapTol))
                                sourceSet.Add(i);
                        }
                    }
                    else if (geom is Mesh extMesh && extMesh.Vertices.Count > 0)
                    {
                        for (int i = 0; i < nV; i++)
                        {
                            Point3d cp = extMesh.ClosestPoint(mesh.Positions[i]);
                            if (cp.DistanceTo(mesh.Positions[i]) <= snapTol * 1.5)
                                sourceSet.Add(i);
                        }
                        if (sourceSet.Count == 0)
                        {
                            int stride = Math.Max(1, extMesh.Vertices.Count / 80);
                            for (int k = 0; k < extMesh.Vertices.Count; k += stride)
                                sourceSet.Add(mesh.FindClosestVertex(extMesh.Vertices.Point3dAt(k)));
                        }
                    }
                    else if (geom is Brep brep && brep.IsValid)
                    {
                        for (int i = 0; i < nV; i++)
                        {
                            Point3d cp = brep.ClosestPoint(mesh.Positions[i]);
                            if (cp.DistanceTo(mesh.Positions[i]) <= snapTol * 1.5)
                                sourceSet.Add(i);
                        }
                        if (sourceSet.Count == 0 && brep.Vertices.Count > 0)
                        {
                            foreach (var bv in brep.Vertices)
                                sourceSet.Add(mesh.FindClosestVertex(bv.Location));
                        }
                    }
                    else
                    {
                        BoundingBox gb = geom.GetBoundingBox(false);
                        if (gb.IsValid)
                            sourceSet.Add(mesh.FindClosestVertex(gb.Center));
                    }
                }
            }

            // Fallback: if no external geometry provided, use naked boundary vertices (or vertex 0)
            if (sourceSet.Count == 0)
            {
                for (int i = 0; i < nV; i++)
                    if (mesh.VertexOnBoundary[i]) sourceSet.Add(i);
                if (sourceSet.Count == 0 && nV > 0)
                    sourceSet.Add(0);
            }

            foreach (int vIdx in sourceSet)
                resolvedSourcePoints.Add(mesh.Positions[vIdx]);

            double[] dist = ComputeHeatDistance(mesh, sourceSet, timeFactor, out Vector3d[] rawFaceGrad, out Vector3d[] rawVertGrad);

            // Smooth & redistribute the scalar gradient via Connection Laplacian so gradient flow is clean and evenly distributed
            smoothVertexGradients = SmoothNRoSyVectorField(mesh, rawVertGrad, Math.Max(0.8, timeFactor), 1, out smoothFaceGradients, out _, out _);
            coloredMesh = ColorMeshByScalar(mesh.RhinoMesh, dist, false);
            return dist;
        }

        // =========================================================================================
        // 23. MULTI-MODE VECTOR & SCALAR GRADIENT SMOOTHING
        //     Mode 0: Connection Laplacian (Vector Heat Parallel Transport)
        //     Mode 1: Helmholtz-Hodge / Eikonal Gradient Redistribution (Curl-Free Uniform Flow)
        //     Mode 2: Crease-Anisotropic Feature-Preserving Vector Diffusion
        //     Mode 3: 4-RoSy / 2-RoSy Symmetric Cross/Line Field Smoothing
        // =========================================================================================
        public static Vector3d[] SmoothVectorAndGradientMultiMode(
            DdgMesh mesh,
            IList<Vector3d> inputVectors,
            IList<double> optionalScalarForGradient,
            double smoothness,
            int mode,
            out Vector3d[] smoothedFaceField,
            out Vector3d[] orthogonalVertexField,
            out double[] redistributedPotential,
            out Mesh coloredMesh)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;

            // Obtain starting tangent vector field (either from inputVectors, or from gradient of optionalScalarForGradient, or principal curvature)
            Vector3d[] rawVertVecs;
            if (inputVectors != null && inputVectors.Count > 0)
            {
                rawVertVecs = mesh.MapVectorInputToVertices(inputVectors);
            }
            else if (optionalScalarForGradient != null && optionalScalarForGradient.Count > 0)
            {
                double[] uIn = mesh.MapScalarInputToVertices(optionalScalarForGradient);
                double[] uSm = SmoothScalarField(mesh, uIn, Math.Max(0.5, smoothness * 0.5), 0.0, out _, out _);
                mesh.ComputeGradient(uSm, out _, out rawVertVecs);
            }
            else
            {
                rawVertVecs = (Vector3d[])mesh.PrincipalDir.Clone();
            }

            redistributedPotential = new double[nV];
            Vector3d[] vSmooth;

            if (mode == 1)
            {
                // MODE 1: Helmholtz-Hodge / Eikonal Gradient Redistribution!
                // 1) Diffuse direction via Connection Laplacian
                Vector3d[] unitDir = SmoothNRoSyVectorField(mesh, rawVertVecs, Math.Max(0.5, smoothness), 1, out Vector3d[] unitFaceDir, out _, out _);

                // 2) Solve Poisson Equation L_c * phi = div(unitFaceDir) to find the globally integrable scalar potential
                //    whose gradient best matches the unit direction field across the entire mesh (equalizing gradient density!)
                double[] div = mesh.ComputeIntegratedDivergence(unitFaceDir);
                var lapBuilder = mesh.BuildCotanLaplacianBuilder(1.0);
                lapBuilder.PinRow(0, 1.0);
                div[0] = 0.0;
                double[] phi = SparseLinearSolver.SolvePCG(lapBuilder.Build(), div, 1200, 1e-8);

                double minP = double.MaxValue;
                for (int i = 0; i < nV; i++) if (phi[i] < minP) minP = phi[i];
                for (int i = 0; i < nV; i++) redistributedPotential[i] = phi[i] - minP;

                // 3) Compute the clean, redistributed gradient of phi and smooth slightly
                mesh.ComputeGradient(redistributedPotential, out smoothedFaceField, out vSmooth);
                for (int i = 0; i < nV; i++)
                    if (vSmooth[i].Length > 1e-12) vSmooth[i].Unitize();
                for (int f = 0; f < nF; f++)
                    if (smoothedFaceField[f].Length > 1e-12) smoothedFaceField[f].Unitize();
            }
            else if (mode == 2)
            {
                // MODE 2: Crease-Anisotropic Feature-Preserving Vector Diffusion
                // Smooths vector components in local tangent frames while stopping across sharp dihedral creases
                vSmooth = (Vector3d[])rawVertVecs.Clone();
                int steps = Math.Max(1, Math.Min(35, (int)Math.Ceiling(smoothness * 4.0)));
                for (int step = 0; step < steps; step++)
                {
                    var nextV = new Vector3d[nV];
                    for (int i = 0; i < nV; i++)
                    {
                        Vector3d ni = mesh.VertexNormals[i];
                        Vector3d sum = vSmooth[i];
                        double wSum = 1.0;

                        foreach (int heIdx in mesh.VertexOutgoingHalfEdges[i])
                        {
                            var he = mesh.HalfEdges[heIdx];
                            int j = he.TargetVertex;
                            var edge = mesh.Edges[he.EdgeIndex];
                            // Strong attenuation across sharp dihedral creases
                            double w = Math.Exp(-6.0 * edge.DihedralAngle * edge.DihedralAngle) * Math.Max(0.05, edge.CotanWeight);
                            // Parallel transport vSmooth[j] to tangent plane at i
                            Vector3d vjProj = vSmooth[j] - (vSmooth[j] * ni) * ni;
                            sum += w * vjProj;
                            wSum += w;
                        }
                        Vector3d res = sum / wSum;
                        res -= (res * ni) * ni;
                        if (res.Length > 1e-12) res.Unitize();
                        nextV[i] = res;
                    }
                    vSmooth = nextV;
                }

                smoothedFaceField = new Vector3d[nF];
                for (int f = 0; f < nF; f++)
                {
                    Vector3d vf = vSmooth[mesh.Faces[f][0]] + vSmooth[mesh.Faces[f][1]] + vSmooth[mesh.Faces[f][2]];
                    vf -= (vf * mesh.FaceNormals[f]) * mesh.FaceNormals[f];
                    if (vf.Length > 1e-12) vf.Unitize();
                    smoothedFaceField[f] = vf;
                }

                // Integrate potential for visualization
                double[] div = mesh.ComputeIntegratedDivergence(smoothedFaceField);
                var lap = mesh.BuildCotanLaplacianBuilder(1.0);
                lap.PinRow(0, 1.0);
                div[0] = 0.0;
                redistributedPotential = SparseLinearSolver.SolvePCG(lap.Build(), div, 800, 1e-7);
            }
            else if (mode == 3)
            {
                // MODE 3: 4-RoSy Cross Field / Symmetric Moving Frame Smooth
                vSmooth = SmoothNRoSyVectorField(mesh, rawVertVecs, smoothness, 4, out smoothedFaceField, out orthogonalVertexField, out _);
                for (int i = 0; i < nV; i++)
                    redistributedPotential[i] = Math.Atan2(vSmooth[i] * mesh.TangentBasisY[i], vSmooth[i] * mesh.TangentBasisX[i]);
                coloredMesh = ColorMeshByScalar(mesh.RhinoMesh, redistributedPotential, false);
                return vSmooth;
            }
            else
            {
                // MODE 0 (Default): Connection Laplacian Vector Heat Diffusion (1-RoSy)
                vSmooth = SmoothNRoSyVectorField(mesh, rawVertVecs, smoothness, 1, out smoothedFaceField, out _, out _);
                double[] div = mesh.ComputeIntegratedDivergence(smoothedFaceField);
                var lap = mesh.BuildCotanLaplacianBuilder(1.0);
                lap.PinRow(0, 1.0);
                div[0] = 0.0;
                redistributedPotential = SparseLinearSolver.SolvePCG(lap.Build(), div, 800, 1e-7);
            }

            orthogonalVertexField = new Vector3d[nV];
            for (int i = 0; i < nV; i++)
            {
                Vector3d ortho = Vector3d.CrossProduct(mesh.VertexNormals[i], vSmooth[i]);
                if (ortho.Length > 1e-12) ortho.Unitize();
                orthogonalVertexField[i] = ortho;
            }

            coloredMesh = ColorMeshByScalar(mesh.RhinoMesh, redistributedPotential, false);
            return vSmooth;
        }

        // =========================================================================================
        // 24. MESH MEDIAL SKELETON & SHRINKING-BALL SPINE (Laplacian Contraction + Tangent-Point Medial Axis)
        // =========================================================================================
        public static List<Polyline> ComputeMeshMedialSkeleton(
            DdgMesh mesh,
            double contractionStrength,
            int skeletonBranches,
            out Mesh contractedSpineMesh,
            out List<Point3d> medialPoints,
            out double[] inscribedRadii,
            out Mesh coloredThicknessMesh)
        {
            int nV = mesh.VertexCount;

            // 1. Compute interior Tangent-Point Shrinking-Ball radius r_in(v) and medial center m_i = x_i - r_in(v) * N_i
            inscribedRadii = ComputeRollingSpheresAndOffset(
                mesh, 0.0, false, 20, out _, out _, out _, out coloredThicknessMesh);

            Point3d[] medialCloud = new Point3d[nV];
            for (int i = 0; i < nV; i++)
            {
                // Blend between Shrinking-Ball center and inward normal contraction
                double r = Math.Min(inscribedRadii[i], mesh.RhinoMesh.GetBoundingBox(false).Diagonal.Length * 0.35);
                medialCloud[i] = mesh.Positions[i] - 0.85 * r * mesh.VertexNormals[i];
            }

            // 2. Implicit Mean-Curvature + Medial-Attraction Contraction:
            //    Solve (W_M + t * L_c) X_skel = W_M * M_medial to collapse the surface mesh onto a clean 1D medial spine!
            double t = Math.Max(0.1, contractionStrength) * 8.0 * mesh.MeanEdgeLength * mesh.MeanEdgeLength;
            var builder = mesh.BuildCotanLaplacianBuilder(t);
            double[] rhsX = new double[nV];
            double[] rhsY = new double[nV];
            double[] rhsZ = new double[nV];

            for (int i = 0; i < nV; i++)
            {
                double mass = mesh.DualAreas[i] * (1.0 + 2.0 * contractionStrength);
                builder.Add(i, i, mass);
                rhsX[i] = mass * medialCloud[i].X;
                rhsY[i] = mass * medialCloud[i].Y;
                rhsZ[i] = mass * medialCloud[i].Z;
            }

            CsrMatrix mat = builder.Build();
            double[] sx = SparseLinearSolver.SolvePCG(mat, rhsX, 800, 1e-7);
            double[] sy = SparseLinearSolver.SolvePCG(mat, rhsY, 800, 1e-7);
            double[] sz = SparseLinearSolver.SolvePCG(mat, rhsZ, 800, 1e-7);

            contractedSpineMesh = mesh.RhinoMesh.DuplicateMesh();
            Point3d[] contractedPts = new Point3d[nV];
            for (int i = 0; i < nV; i++)
            {
                contractedPts[i] = new Point3d(sx[i], sy[i], sz[i]);
                contractedSpineMesh.Vertices.SetVertex(i, contractedPts[i]);
            }
            contractedSpineMesh.Normals.ComputeNormals();

            // 3. Extract clean 1D Skeleton Graph Curves by clustering contracted spine nodes via Farthest-Point Geodesic Sampling
            //    and connecting them along the mesh's intrinsic topology!
            int kNodes = Math.Max(4, Math.Min(80, skeletonBranches * 4));
            var nodeIndices = new List<int>(kNodes);
            var minDist = new double[nV];
            for (int i = 0; i < nV; i++) minDist[i] = double.MaxValue;

            // Start from vertex with largest inscribed radius (deepest core of the shape)
            int rootV = 0;
            double maxR = -1.0;
            for (int i = 0; i < nV; i++)
            {
                if (inscribedRadii[i] > maxR) { maxR = inscribedRadii[i]; rootV = i; }
            }
            nodeIndices.Add(rootV);

            for (int k = 1; k < kNodes; k++)
            {
                int prevV = nodeIndices[k - 1];
                Point3d pPrev = contractedPts[prevV];
                int farthest = 0;
                double bestD = -1.0;
                for (int i = 0; i < nV; i++)
                {
                    double d = contractedPts[i].DistanceTo(pPrev);
                    if (d < minDist[i]) minDist[i] = d;
                    if (minDist[i] > bestD) { bestD = minDist[i]; farthest = i; }
                }
                if (bestD < 1e-5) break;
                nodeIndices.Add(farthest);
            }

            medialPoints = new List<Point3d>(nodeIndices.Count);
            foreach (int idx in nodeIndices)
                medialPoints.Add(contractedPts[idx]);

            // Build Minimum Spanning Tree (MST) + Cycle Closures over the medial nodes to form smooth skeleton polylines
            int m = medialPoints.Count;
            var skelLines = new List<Line>();
            if (m >= 2)
            {
                bool[] inTree = new bool[m];
                inTree[0] = true;
                for (int step = 1; step < m; step++)
                {
                    int bestA = -1, bestB = -1;
                    double bestLen = double.MaxValue;
                    for (int a = 0; a < m; a++)
                    {
                        if (!inTree[a]) continue;
                        for (int b = 0; b < m; b++)
                        {
                            if (inTree[b]) continue;
                            double d = medialPoints[a].DistanceTo(medialPoints[b]);
                            if (d < bestLen)
                            {
                                bestLen = d;
                                bestA = a;
                                bestB = b;
                            }
                        }
                    }
                    if (bestA != -1 && bestB != -1)
                    {
                        inTree[bestB] = true;
                        skelLines.Add(new Line(medialPoints[bestA], medialPoints[bestB]));
                    }
                }
            }

            var skeletonPolylines = new List<Polyline>();
            if (skelLines.Count > 0)
            {
                Curve[] joined = Curve.JoinCurves(skelLines.ConvertAll(l => (Curve)new LineCurve(l)), mesh.MeanEdgeLength * 0.1);
                if (joined != null)
                {
                    foreach (var c in joined)
                        if (c.TryGetPolyline(out Polyline pl))
                            skeletonPolylines.Add(pl);
                }
            }

            return skeletonPolylines;
        }

        // =========================================================================================
        // 25. 3D HEAT ISOSURFACE, VOLUMETRIC BOOLEAN UNION & VECTOR FIELD ISOSURFACE
        //     (Signed Heat Method 2024 + Marching Tetrahedra Isosurface Extraction)
        // =========================================================================================
        public static Mesh Compute3DHeatIsosurfaceAndOffset(
            IList<GeometryBase> inputGeometries,
            double offsetIsoDistance,
            IList<Vector3d> optional3DVectorField,
            int gridResolution,
            out List<Box> volumetricIsoBoxes,
            out List<double> gridFieldValues)
        {
            volumetricIsoBoxes = new List<Box>();
            gridFieldValues = new List<double>();

            if (inputGeometries == null || inputGeometries.Count == 0)
                return new Mesh();

            // 1. Compute combined 3D Bounding Box expanded by 1.6 * |offsetIsoDistance| so the offset isosurface closes cleanly!
            BoundingBox bbox = BoundingBox.Empty;
            var meshes = new List<Mesh>();
            var curves = new List<Curve>();
            var points = new List<Point3d>();

            foreach (var g in inputGeometries)
            {
                if (g == null) continue;
                bbox.Union(g.GetBoundingBox(false));
                if (g is Mesh m && m.Vertices.Count > 0) meshes.Add(m);
                else if (g is Curve c && c.IsValid) curves.Add(c);
                else if (g is Brep b && b.IsValid)
                {
                    Mesh[] bm = Mesh.CreateFromBrep(b, MeshingParameters.FastRenderMesh);
                    if (bm != null) meshes.AddRange(bm);
                }
                else if (g is Rhino.Geometry.Point pt) points.Add(pt.Location);
            }

            if (!bbox.IsValid) return new Mesh();

            double diag = Math.Max(1.0, bbox.Diagonal.Length);
            double isoTarget = Math.Abs(offsetIsoDistance) > 1e-5 ? Math.Abs(offsetIsoDistance) : 0.08 * diag;
            double pad = isoTarget * 1.65 + 0.06 * diag;
            Point3d minPt = bbox.Min - new Vector3d(pad, pad, pad);
            Point3d maxPt = bbox.Max + new Vector3d(pad, pad, pad);

            int nx = Math.Max(8, Math.Min(38, gridResolution));
            int ny = nx;
            int nz = nx;

            double dx = (maxPt.X - minPt.X) / (nx - 1);
            double dy = (maxPt.Y - minPt.Y) / (ny - 1);
            double dz = (maxPt.Z - minPt.Z) / (nz - 1);

            int totalGridPts = nx * ny * nz;
            Point3d[] gridPts = new Point3d[totalGridPts];
            double[] rawField = new double[totalGridPts];

            // Soft-Min Smooth Heat Kernel Parameter (Log-Sum-Exp Heat Kernel Union: phi_union = -tau * ln(sum exp(-d_k / tau))!)
            // This produces Keenan Crane's exact Varadhan Heat Kernel smooth Boolean union of multiple meshes/curves/points!
            double tau = Math.Max(1e-4, 0.22 * isoTarget);
            bool hasVecField = optional3DVectorField != null && optional3DVectorField.Count > 0;

            for (int ix = 0; ix < nx; ix++)
            {
                for (int iy = 0; iy < ny; iy++)
                {
                    for (int iz = 0; iz < nz; iz++)
                    {
                        int idx = (ix * ny + iy) * nz + iz;
                        Point3d p = new Point3d(minPt.X + ix * dx, minPt.Y + iy * dy, minPt.Z + iz * dz);
                        gridPts[idx] = p;

                        // Evaluate smooth Heat-Kernel distance to all input geometries (Smooth Boolean Union!)
                        double minD = double.MaxValue;
                        var dists = new List<double>();

                        foreach (var m in meshes)
                        {
                            Point3d cp = m.ClosestPoint(p);
                            double d = p.DistanceTo(cp);
                            // If mesh is closed and point is inside, make distance negative (Signed Distance!)
                            if (m.IsClosed && m.IsPointInside(p, 1e-5, false))
                                d = -d;
                            dists.Add(d);
                            if (d < minD) minD = d;
                        }

                        foreach (var c in curves)
                        {
                            if (c.ClosestPoint(p, out double tParam))
                            {
                                double d = p.DistanceTo(c.PointAt(tParam));
                                dists.Add(d);
                                if (d < minD) minD = d;
                            }
                        }

                        foreach (var pt in points)
                        {
                            double d = p.DistanceTo(pt);
                            dists.Add(d);
                            if (d < minD) minD = d;
                        }

                        double val;
                        if (dists.Count > 1)
                        {
                            // Smooth Heat-Kernel Log-Sum-Exp Boolean Union!
                            double sumExp = 0.0;
                            foreach (double d in dists)
                                sumExp += Math.Exp(-(d - minD) / tau);
                            val = minD - tau * Math.Log(Math.Max(1e-12, sumExp));
                        }
                        else if (dists.Count == 1)
                        {
                            val = minD;
                        }
                        else
                        {
                            val = p.DistanceTo(bbox.Center);
                        }

                        // Optional 3D Vector Field modulation (Signed Heat Method 3D vector divergence/potential coupling)
                        if (hasVecField)
                        {
                            Vector3d v3 = optional3DVectorField[idx % optional3DVectorField.Count];
                            if (v3.IsValid && v3.Length > 1e-9)
                            {
                                Vector3d rel = p - bbox.Center;
                                val += 0.25 * isoTarget * (rel * v3) / (diag * v3.Length);
                            }
                        }

                        rawField[idx] = val - isoTarget;
                    }
                }
            }

            // 2. Perform 1 pass of 3D Discrete Laplacian Heat Diffusion on the 3D grid so the isosurface is C2-smooth!
            double[] smoothField = new double[totalGridPts];
            for (int ix = 0; ix < nx; ix++)
            {
                for (int iy = 0; iy < ny; iy++)
                {
                    for (int iz = 0; iz < nz; iz++)
                    {
                        int idx = (ix * ny + iy) * nz + iz;
                        if (ix == 0 || ix == nx - 1 || iy == 0 || iy == ny - 1 || iz == 0 || iz == nz - 1)
                        {
                            smoothField[idx] = rawField[idx];
                            continue;
                        }
                        double nbSum = rawField[((ix - 1) * ny + iy) * nz + iz]
                                     + rawField[((ix + 1) * ny + iy) * nz + iz]
                                     + rawField[(ix * ny + (iy - 1)) * nz + iz]
                                     + rawField[(ix * ny + (iy + 1)) * nz + iz]
                                     + rawField[(ix * ny + iy) * nz + (iz - 1)]
                                     + rawField[(ix * ny + iy) * nz + (iz + 1)];
                        smoothField[idx] = 0.65 * rawField[idx] + 0.35 * (nbSum / 6.0);
                    }
                }
            }

            gridFieldValues.AddRange(smoothField);

            // 3. Extract Watertight Isosurface Mesh at smoothField == 0 using Kuhn 6-Tetrahedra Marching Tetrahedra!
            var isoMesh = new Mesh();
            int[][] kuhnTets = new int[][]
            {
                new int[] { 0, 1, 3, 7 },
                new int[] { 0, 1, 5, 7 },
                new int[] { 0, 4, 5, 7 },
                new int[] { 0, 4, 6, 7 },
                new int[] { 0, 2, 6, 7 },
                new int[] { 0, 2, 3, 7 }
            };

            for (int ix = 0; ix < nx - 1; ix++)
            {
                for (int iy = 0; iy < ny - 1; iy++)
                {
                    for (int iz = 0; iz < nz - 1; iz++)
                    {
                        int[] cIdx = new int[8]
                        {
                            (ix * ny + iy) * nz + iz,
                            ((ix + 1) * ny + iy) * nz + iz,
                            (ix * ny + (iy + 1)) * nz + iz,
                            ((ix + 1) * ny + (iy + 1)) * nz + iz,
                            (ix * ny + iy) * nz + (iz + 1),
                            ((ix + 1) * ny + iy) * nz + (iz + 1),
                            (ix * ny + (iy + 1)) * nz + (iz + 1),
                            ((ix + 1) * ny + (iy + 1)) * nz + (iz + 1)
                        };

                        // Check if voxel is inside or straddling the isosurface
                        bool anyInside = false, anyOutside = false;
                        for (int c = 0; c < 8; c++)
                        {
                            if (smoothField[cIdx[c]] <= 0.0) anyInside = true;
                            else anyOutside = true;
                        }

                        if (anyInside && volBoxCountLimit(volumetricIsoBoxes.Count) && (ix + iy + iz) % 2 == 0)
                        {
                            Point3d p0 = gridPts[cIdx[0]];
                            Point3d p7 = gridPts[cIdx[7]];
                            volumetricIsoBoxes.Add(new Box(new BoundingBox(p0, p7)));
                        }

                        if (!anyInside || !anyOutside) continue;

                        // March each of the 6 Kuhn tetrahedra inside the cube
                        for (int t = 0; t < 6; t++)
                        {
                            int i0 = cIdx[kuhnTets[t][0]];
                            int i1 = cIdx[kuhnTets[t][1]];
                            int i2 = cIdx[kuhnTets[t][2]];
                            int i3 = cIdx[kuhnTets[t][3]];

                            AddMarchingTetIsoTriangles(
                                isoMesh,
                                gridPts[i0], gridPts[i1], gridPts[i2], gridPts[i3],
                                smoothField[i0], smoothField[i1], smoothField[i2], smoothField[i3]);
                        }
                    }
                }
            }

            isoMesh.Weld(Math.PI);
            isoMesh.Compact();
            isoMesh.UnifyNormals();
            isoMesh.Normals.ComputeNormals();
            return isoMesh;
        }

        private static bool volBoxCountLimit(int current) => current < 600;

        private static void AddMarchingTetIsoTriangles(
            Mesh target,
            Point3d p0, Point3d p1, Point3d p2, Point3d p3,
            double v0, double v1, double v2, double v3)
        {
            Point3d[] p = { p0, p1, p2, p3 };
            double[] v = { v0, v1, v2, v3 };

            var crossPts = new List<Point3d>(4);
            int[,] edges = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };

            for (int e = 0; e < 6; e++)
            {
                int a = edges[e, 0], b = edges[e, 1];
                if ((v[a] < 0.0 && v[b] >= 0.0) || (v[b] < 0.0 && v[a] >= 0.0))
                {
                    double denom = v[a] - v[b];
                    double t = Math.Abs(denom) > 1e-14 ? v[a] / denom : 0.5;
                    crossPts.Add(p[a] + t * (p[b] - p[a]));
                }
            }

            if (crossPts.Count == 3)
            {
                int baseV = target.Vertices.Count;
                target.Vertices.Add(crossPts[0]);
                target.Vertices.Add(crossPts[1]);
                target.Vertices.Add(crossPts[2]);
                target.Faces.AddFace(baseV, baseV + 1, baseV + 2);
            }
            else if (crossPts.Count == 4)
            {
                int baseV = target.Vertices.Count;
                target.Vertices.Add(crossPts[0]);
                target.Vertices.Add(crossPts[1]);
                target.Vertices.Add(crossPts[2]);
                target.Vertices.Add(crossPts[3]);
                target.Faces.AddFace(baseV, baseV + 1, baseV + 2);
                target.Faces.AddFace(baseV + 1, baseV + 3, baseV + 2);
            }
        }

        // =========================================================================================
        // 26. COLLISION-AVOIDING SO(3) ROBOT / CNC PLANE SMOOTHING & STABILIZATION
        //     Eliminates drastic 5/6-axis tool flips and tilts planes away from mesh concavities!
        // =========================================================================================
        public static List<Plane> ComputeCollisionFreeSmoothPlanes(
            DdgMesh mesh,
            IList<Plane> inputPlanes,
            double planeSmoothness,
            double toolClearanceRadius,
            out List<Point3d> collisionAdjustedPoints,
            out double[] tiltCorrectionDegrees)
        {
            int count = inputPlanes != null ? inputPlanes.Count : 0;
            collisionAdjustedPoints = new List<Point3d>();
            tiltCorrectionDegrees = new double[count];
            var outPlanes = new List<Plane>(count);
            if (count == 0) return outPlanes;

            // 1. Compute exterior Rolling-Sphere Accessibility field on the mesh to detect collision/concave traps
            double rCheck = Math.Max(mesh.MeanEdgeLength, Math.Abs(toolClearanceRadius));
            double[] rMax = ComputeRollingSpheresAndOffset(mesh, rCheck, true, 10, out _, out _, out _, out _);
            mesh.ComputeGradient(rMax, out _, out Vector3d[] escapeGrad);

            Point3d[] origins = new Point3d[count];
            Vector3d[] zAxes = new Vector3d[count];
            Vector3d[] xAxes = new Vector3d[count];

            BoundingBox bbox = mesh.RhinoMesh.GetBoundingBox(false);
            Point3d meshCenter = bbox.Center;

            // 2. Apply Repulsive Collision-Avoidance Tilt where local rolling sphere radius < rCheck
            for (int i = 0; i < count; i++)
            {
                Plane pl = inputPlanes[i];
                origins[i] = pl.Origin;
                Vector3d z = pl.ZAxis;
                if (z.Length < 1e-12) z = Vector3d.ZAxis;
                z.Unitize();

                int vNear = mesh.FindClosestVertex(pl.Origin);
                if (rMax[vNear] < rCheck)
                {
                    // Concave collision danger! Tilt tool Z-axis toward the escape gradient (higher accessibility) + outward radial direction
                    Vector3d esc = escapeGrad[vNear];
                    if (esc.Length > 1e-12) esc.Unitize();
                    Vector3d radial = pl.Origin - meshCenter;
                    if (radial.Length > 1e-12) radial.Unitize();

                    double severity = Math.Min(1.0, (rCheck - rMax[vNear]) / Math.Max(1e-6, rCheck));
                    z += severity * (0.65 * esc + 0.45 * radial);
                    if (z.Length > 1e-12) z.Unitize();
                    collisionAdjustedPoints.Add(pl.Origin);
                }

                zAxes[i] = z;
                xAxes[i] = pl.XAxis.Length > 1e-12 ? pl.XAxis : Vector3d.XAxis;
            }

            // 3. Smooth Z-axes (Tool Axis Tilt) and X-axes (Tool Roll) along the trajectory via Lie-Group / Parallel-Transport averaging
            int passes = Math.Max(1, Math.Min(40, (int)Math.Ceiling(planeSmoothness * 4.0)));
            Vector3d[] smoothZ = (Vector3d[])zAxes.Clone();
            for (int pass = 0; pass < passes; pass++)
            {
                var nextZ = new Vector3d[count];
                for (int i = 0; i < count; i++)
                {
                    int prev = Math.Max(0, i - 1);
                    int next = Math.Min(count - 1, i + 1);
                    Vector3d avg = 0.25 * smoothZ[prev] + 0.50 * smoothZ[i] + 0.25 * smoothZ[next];
                    if (avg.Length > 1e-12) avg.Unitize();
                    else avg = smoothZ[i];
                    nextZ[i] = avg;
                }
                smoothZ = nextZ;
            }

            // 4. Reconstruct orthonormal SE(3) Planes via Levi-Civita Parallel Transport (eliminates sudden wrist J4/J6 flips!)
            Vector3d prevX = xAxes[0] - (xAxes[0] * smoothZ[0]) * smoothZ[0];
            if (prevX.Length < 1e-10)
                prevX = Vector3d.CrossProduct(smoothZ[0], Math.Abs(smoothZ[0].X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
            prevX.Unitize();

            for (int i = 0; i < count; i++)
            {
                Vector3d z = smoothZ[i];
                // Blend parallel-transported previous X-axis with smoothed input X-axis
                Vector3d ptX = prevX - (prevX * z) * z;
                if (ptX.Length < 1e-10)
                    ptX = Vector3d.CrossProduct(z, Math.Abs(z.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                ptX.Unitize();

                prevX = ptX;
                Vector3d y = Vector3d.CrossProduct(z, ptX);
                y.Unitize();

                double cosAng = Math.Max(-1.0, Math.Min(1.0, z * zAxes[i]));
                tiltCorrectionDegrees[i] = Math.Acos(cosAng) * (180.0 / Math.PI);

                outPlanes.Add(new Plane(origins[i], ptX, y));
            }

            return outPlanes;
        }

        // =========================================================================================
        // 27. REPULSIVE SHELLS & 2D CONFIGURATION-SPACE (C-SPACE) GEODESIC ROBOT TOOLPATH
        //     Inspired by "Repulsive Shells" (Sassen, Schumacher, Rumpf, Wardetzky, Crane 2024)
        //     & "Repulsive Curves" (Yu, Schumacher, Crane 2021).
        //     - Mode 0: Fixed TCP along Printing/Milling Path + Spindle/Plane Collision & Singularity Avoidance
        //     - Mode 1: Pick & Place 3D Obstacle-Avoiding Geodesic Trajectory from A to B
        //     - Mode 2: 2D Spherical Orientation Map [Theta, Phi] for Spindle & Wrist Singularity Analysis
        // =========================================================================================
        public static Polyline ComputeRepulsiveShellsAndCSpaceRobotPath(
            IList<Plane> inputFrames,
            IList<Mesh> obstacleMeshes,
            double spindleRadiusInput,
            int mode,
            out List<Plane> optimalRobotPlanes,
            out Mesh cSpaceMapMesh,
            out Polyline cSpaceGeodesicPath,
            out Mesh spindleShellPreviewMesh,
            out double[] clearanceMarginPerStep)
        {
            optimalRobotPlanes = new List<Plane>();
            cSpaceMapMesh = new Mesh();
            cSpaceGeodesicPath = new Polyline();
            spindleShellPreviewMesh = new Mesh();
            clearanceMarginPerStep = Array.Empty<double>();

            if (inputFrames == null || inputFrames.Count == 0)
                return new Polyline();

            // 1. Resample or interpolate input frames into nSteps = 44 trajectory stations from A to B
            int nSteps = 44;
            int nAngles = 29; // V-resolution of the 2D Configuration-Space Map
            var baseFrames = new Plane[nSteps];

            if (inputFrames.Count == 1)
            {
                for (int i = 0; i < nSteps; i++) baseFrames[i] = inputFrames[0];
            }
            else
            {
                // Compute cumulative arc length
                double[] cumLen = new double[inputFrames.Count];
                for (int i = 1; i < inputFrames.Count; i++)
                {
                    double d = inputFrames[i].Origin.DistanceTo(inputFrames[i - 1].Origin);
                    cumLen[i] = cumLen[i - 1] + Math.Max(1e-4, d);
                }
                double totalL = cumLen[inputFrames.Count - 1];

                for (int s = 0; s < nSteps; s++)
                {
                    double targetS = (totalL * s) / (nSteps - 1.0);
                    int idx = 0;
                    while (idx < inputFrames.Count - 2 && cumLen[idx + 1] < targetS)
                        idx++;

                    double segL = Math.Max(1e-8, cumLen[idx + 1] - cumLen[idx]);
                    double t = Math.Max(0.0, Math.Min(1.0, (targetS - cumLen[idx]) / segL));

                    Plane p0 = inputFrames[idx];
                    Plane p1 = inputFrames[idx + 1];
                    Point3d orig = (1.0 - t) * p0.Origin + t * p1.Origin;
                    Vector3d z = (1.0 - t) * p0.ZAxis + t * p1.ZAxis;
                    if (z.Length < 1e-10) z = Vector3d.ZAxis;
                    z.Unitize();
                    Vector3d x = (1.0 - t) * p0.XAxis + t * p1.XAxis;
                    x -= (x * z) * z;
                    if (x.Length < 1e-10)
                        x = Vector3d.CrossProduct(z, Math.Abs(z.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                    x.Unitize();
                    Vector3d y = Vector3d.CrossProduct(z, x);
                    y.Unitize();
                    baseFrames[s] = new Plane(orig, x, y);
                }
            }

            // 2. Prepare External Collision Obstacle Meshes for Exact Surface Closest-Point Spindle Evaluation
            BoundingBox sceneBox = new BoundingBox(baseFrames[0].Origin, baseFrames[nSteps - 1].Origin);
            for (int i = 0; i < nSteps; i++) sceneBox.Union(baseFrames[i].Origin);

            var validObsMeshes = new List<Mesh>();
            var obsPts = new List<Point3d>();

            if (obstacleMeshes != null)
            {
                foreach (Mesh om in obstacleMeshes)
                {
                    if (om == null || om.Vertices.Count == 0) continue;
                    om.Normals.ComputeNormals();
                    sceneBox.Union(om.GetBoundingBox(false));
                    validObsMeshes.Add(om);

                    int fStride = Math.Max(1, om.Faces.Count / 120);
                    for (int f = 0; f < om.Faces.Count; f += fStride)
                        obsPts.Add(om.Faces.GetFaceCenter(f));
                }
            }

            double diag = Math.Max(10.0, sceneBox.Diagonal.Length);
            double rSpindle = spindleRadiusInput > 1e-5 ? spindleRadiusInput : diag * 0.065;
            double spindleLen = rSpindle * 4.2;

            // Define robot wrist singularity axis (inline forearm alignment or downward self-collision cone)
            Vector3d wristSingularityAxis = new Vector3d(0.0, -0.85, -0.52);
            wristSingularityAxis.Unitize();
            Point3d robotBaseColumn = sceneBox.Center - new Vector3d(0.0, diag * 0.65, 0.0);

            // Helper: Evaluate Repulsive Shell Energy for a candidate Plane against External Obstacles + IK Singularity
            double EvaluateSpindleShellEnergy(Point3d tcp, Vector3d zAxis, out double minClearance)
            {
                zAxis.Unitize();
                Vector3d uAx = Vector3d.CrossProduct(zAxis, Math.Abs(zAxis.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                uAx.Unitize();
                Vector3d vAx = Vector3d.CrossProduct(zAxis, uAx);
                vAx.Unitize();

                // Sample 7 spheres along the Spindle Shell + Tool Holder Plane Disk (lifted off TCP so only external obstacles collide)
                Point3d[] shellPts = new Point3d[7];
                double[] shellRadii = new double[7];

                shellPts[0] = tcp + zAxis * (0.65 * rSpindle);
                shellRadii[0] = 0.35 * rSpindle;

                shellPts[1] = tcp + zAxis * (1.45 * rSpindle);
                shellRadii[1] = 0.85 * rSpindle;

                shellPts[2] = tcp + zAxis * (2.75 * rSpindle);
                shellRadii[2] = 1.00 * rSpindle;

                shellPts[3] = tcp + zAxis * spindleLen;
                shellRadii[3] = 1.05 * rSpindle;

                // Tool holder flange disk (checks if the Plane Holder itself collides with external obstacles!)
                Point3d flangeCenter = tcp + zAxis * (1.60 * rSpindle);
                shellPts[4] = flangeCenter + uAx * (1.25 * rSpindle);
                shellRadii[4] = 0.45 * rSpindle;
                shellPts[5] = flangeCenter - uAx * (1.25 * rSpindle);
                shellRadii[5] = 0.45 * rSpindle;
                shellPts[6] = flangeCenter + vAx * (1.25 * rSpindle);
                shellRadii[6] = 0.45 * rSpindle;

                minClearance = diag;
                double energy = 0.0;

                // A. Exact Surface Closest-Point Repulsive Shell Energy against External Obstacle Meshes
                int sStart = (mode == 0) ? 1 : 0; // In Mode 0 (TCP fixed on path), only check Spindle Body & Holder Plane (s=1..6)
                for (int mIdx = 0; mIdx < validObsMeshes.Count; mIdx++)
                {
                    Mesh om = validObsMeshes[mIdx];
                    bool isClosedObs = om.IsClosed;

                    for (int s = sStart; s < 7; s++)
                    {
                        Point3d sp = shellPts[s];
                        Point3d cp = om.ClosestPoint(sp);
                        double distToSurf = sp.DistanceTo(cp);

                        if (isClosedObs && distToSurf < 3.0 * rSpindle && om.IsPointInside(sp, 1e-4, false))
                            distToSurf = -distToSurf;

                        double d = distToSurf - shellRadii[s];
                        if (d < minClearance) minClearance = d;

                        if (d <= 0.0)
                        {
                            // Hard external obstacle collision barrier
                            double pen = Math.Abs(d) / Math.Max(1e-5, rSpindle);
                            energy += 18.0 + 48.0 * pen * pen;
                        }
                        else if (d < 2.2 * rSpindle)
                        {
                            // Smooth Tangent-Point repulsive shell falloff near external obstacle
                            double ratio = (2.2 * rSpindle - d) / (2.2 * rSpindle);
                            energy += 4.5 * ratio * ratio * ratio;
                        }
                    }
                }

                // B. Embedded Analytical 6-Axis Robot Inverse Kinematics (IK) Singularity & Self-Collision Check
                //    Models a standard 6-DOF spherical-wrist robot arm (Shoulder L1, Upper Arm L2, Forearm L3)
                Point3d wristCenter = tcp + zAxis * spindleLen;
                Point3d shoulderJoint = robotBaseColumn + new Vector3d(0.0, 0.0, 0.35 * diag);
                double L2 = 0.72 * diag; // Upper arm length
                double L3 = 0.68 * diag; // Forearm length

                Vector3d sw = wristCenter - shoulderJoint;
                double rHoriz = Math.Sqrt(sw.X * sw.X + sw.Y * sw.Y);
                double dReach = sw.Length;

                // 1) Overhead Waist Singularity (J1 undefined when wrist passes directly over shoulder axis rHoriz -> 0)
                if (rHoriz < 1.2 * rSpindle)
                {
                    double ohPen = (1.2 * rSpindle - rHoriz) / Math.Max(1e-5, rSpindle);
                    energy += 9.0 * ohPen * ohPen;
                }

                // 2) Elbow Reach / Workspace Boundary Singularity (Law of Cosines cos(theta3) -> +-1)
                double maxReach = 0.96 * (L2 + L3);
                double minReach = 1.15 * Math.Abs(L2 - L3);
                if (dReach > maxReach)
                {
                    double rPen = (dReach - maxReach) / Math.Max(1e-5, rSpindle);
                    energy += 15.0 * rPen * rPen;
                }
                else if (dReach < minReach)
                {
                    double rPen = (minReach - dReach) / Math.Max(1e-5, rSpindle);
                    energy += 12.0 * rPen * rPen;
                }

                // Solve analytical 2-Link Elbow Position & Forearm Axis a_forearm
                double cosAlpha = Math.Max(-0.999, Math.Min(0.999, (L2 * L2 + dReach * dReach - L3 * L3) / Math.Max(1e-6, 2.0 * L2 * dReach)));
                double sinAlpha = Math.Sqrt(Math.Max(0.0, 1.0 - cosAlpha * cosAlpha));
                Vector3d swDir = dReach > 1e-6 ? (sw / dReach) : Vector3d.YAxis;
                Vector3d armUpPlane = Vector3d.CrossProduct(Vector3d.CrossProduct(swDir, Vector3d.ZAxis), swDir);
                if (armUpPlane.Length < 1e-8) armUpPlane = Vector3d.ZAxis;
                armUpPlane.Unitize();

                Point3d elbowJoint = shoulderJoint + L2 * (cosAlpha * swDir + sinAlpha * armUpPlane);
                Vector3d forearmAxis = wristCenter - elbowJoint;
                if (forearmAxis.Length > 1e-8) forearmAxis.Unitize();
                else forearmAxis = wristSingularityAxis;

                // 3) Exact Wrist Singularity (J5 -> 0 when Spindle Axis zAxis aligns with Forearm Axis!)
                //    Note: Spindle Z points from TCP to Wrist (+zAxis), while forearm points from Elbow to Wrist (+forearmAxis)
                double cosTheta5 = Math.Abs(zAxis * forearmAxis);
                if (cosTheta5 > 0.82) // within ~34 degrees of collinear J4-J6 wrist lock
                {
                    double wPen = (cosTheta5 - 0.82) / 0.18;
                    energy += 14.0 * wPen * wPen;
                }

                // 4) Prevent upside-down tool flips (zAxis.Z < -0.15) & Base Self-Collision
                if (zAxis.Z < -0.15)
                {
                    double flipPen = (-0.15 - zAxis.Z);
                    energy += 14.0 * flipPen * flipPen;
                }
                double distBase = wristCenter.DistanceTo(robotBaseColumn);
                if (distBase < 2.2 * rSpindle)
                {
                    double bPen = (2.2 * rSpindle - distBase) / Math.Max(1e-5, rSpindle);
                    energy += 10.0 * bPen * bPen;
                }

                return energy;
            }

            // 3. Build the 2D Configuration-Space Energy Matrix E[i, j] and candidate states
            double[,] cSpaceEnergy = new double[nSteps, nAngles];
            double[,] cSpaceClearance = new double[nSteps, nAngles];
            Point3d[,] candOrigins = new Point3d[nSteps, nAngles];
            Vector3d[,] candZAxes = new Vector3d[nSteps, nAngles];

            // Compute global arch direction for Mode 1 (Pick & Place obstacle avoidance)
            Vector3d abVec = baseFrames[nSteps - 1].Origin - baseFrames[0].Origin;
            if (abVec.Length < 1e-6) abVec = Vector3d.XAxis;
            abVec.Unitize();
            Vector3d upArch = Vector3d.ZAxis - (Vector3d.ZAxis * abVec) * abVec;
            if (upArch.Length < 1e-6) upArch = Vector3d.YAxis;
            upArch.Unitize();
            Vector3d sideArch = Vector3d.CrossProduct(upArch, abVec);
            sideArch.Unitize();

            double maxTiltRad = 72.0 * (Math.PI / 180.0);

            for (int i = 0; i < nSteps; i++)
            {
                double u = i / (double)(nSteps - 1);
                Plane bf = baseFrames[i];
                Vector3d z0 = bf.ZAxis;
                z0.Unitize();

                // Local tangent along path
                Vector3d tang = (i < nSteps - 1)
                    ? (baseFrames[i + 1].Origin - bf.Origin)
                    : (bf.Origin - baseFrames[Math.Max(0, i - 1)].Origin);
                if (tang.Length < 1e-8) tang = bf.XAxis;
                tang.Unitize();

                Vector3d side = Vector3d.CrossProduct(z0, tang);
                if (side.Length < 1e-8)
                    side = Vector3d.CrossProduct(z0, Math.Abs(z0.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                side.Unitize();
                Vector3d fwd = Vector3d.CrossProduct(side, z0);
                fwd.Unitize();

                // Smooth arch envelope sin(pi * u) that is 0 at endpoints A and B
                double archEnv = Math.Sin(Math.PI * u);

                for (int j = 0; j < nAngles; j++)
                {
                    // Normalized parameter v in [-1, +1]
                    double v = (2.0 * j) / (nAngles - 1.0) - 1.0;

                    if (mode == 1)
                    {
                        // MODE 1: Pick & Place 3D Trajectory (A to B with Obstacles)
                        // v explores a 3D arch + lateral bypass corridor around obstacles while keeping A and B fixed!
                        double liftH = (0.38 * diag) * archEnv * (0.45 + 0.55 * Math.Abs(v));
                        double sideOffset = (0.35 * diag) * archEnv * v;
                        Point3d pos = bf.Origin + upArch * liftH + sideArch * sideOffset;

                        // Automatically tilt the spindle away from the obstacle center as it bypasses
                        Vector3d zCand = z0 + (0.45 * archEnv * v) * sideArch + (0.25 * archEnv) * upArch;
                        zCand.Unitize();

                        double e = EvaluateSpindleShellEnergy(pos, zCand, out double clr);
                        // Also test slight forward/backward tilt at this position and keep the cleaner orientation
                        Vector3d zAlt = zCand + (0.30 * archEnv) * abVec;
                        zAlt.Unitize();
                        double eAlt = EvaluateSpindleShellEnergy(pos, zAlt, out double clrAlt);
                        if (eAlt < e)
                        {
                            e = eAlt;
                            clr = clrAlt;
                            zCand = zAlt;
                        }

                        candOrigins[i, j] = pos;
                        candZAxes[i, j] = zCand;
                        cSpaceEnergy[i, j] = e;
                        cSpaceClearance[i, j] = clr;
                    }
                    else if (mode == 2)
                    {
                        // MODE 2: 2D Spherical Orientation Cone [Theta, Phi]
                        double theta = (u * 2.0 - 1.0) * maxTiltRad;
                        double phi = v * maxTiltRad;
                        Vector3d zCand = Math.Cos(theta) * Math.Cos(phi) * z0
                                       + Math.Sin(theta) * side
                                       + Math.Cos(theta) * Math.Sin(phi) * fwd;
                        zCand.Unitize();

                        // Evaluate integrated energy along the trajectory for this spherical angle + at station i
                        double e = EvaluateSpindleShellEnergy(bf.Origin, zCand, out double clr);
                        candOrigins[i, j] = bf.Origin;
                        candZAxes[i, j] = zCand;
                        cSpaceEnergy[i, j] = e;
                        cSpaceClearance[i, j] = clr;
                    }
                    else
                    {
                        // MODE 0 (Default): Fixed TCP on Printing/Milling Path + Spindle Tilt C-Space Geodesic
                        double tiltAngle = v * maxTiltRad;
                        Vector3d zSide = Math.Cos(tiltAngle) * z0 + Math.Sin(tiltAngle) * side;
                        zSide.Unitize();

                        double bestE = EvaluateSpindleShellEnergy(bf.Origin, zSide, out double bestClr);
                        Vector3d bestZ = zSide;

                        // Also check lead/lag tilt along tangent so spindle can dodge obstacles ahead/behind on the path
                        for (int leadStep = -1; leadStep <= 1; leadStep += 2)
                        {
                            Vector3d zLead = zSide + (leadStep * 0.28 * Math.Abs(v)) * fwd;
                            zLead.Unitize();
                            double eLead = EvaluateSpindleShellEnergy(bf.Origin, zLead, out double clrLead);
                            if (eLead + 0.05 < bestE)
                            {
                                bestE = eLead;
                                bestClr = clrLead;
                                bestZ = zLead;
                            }
                        }

                        candOrigins[i, j] = bf.Origin;
                        candZAxes[i, j] = bestZ;
                        cSpaceEnergy[i, j] = bestE;
                        cSpaceClearance[i, j] = bestClr;
                    }
                }
            }

            // 4. Solve the Global Riemannian Geodesic Path from A (i=0) to B (i=nSteps-1) across the 2D C-Space Manifold
            //    Minimizing: sum_i [ w_elastic * ||q_{i+1} - q_i||^2 + w_repulsive * E(i, j_i) ]
            double[,] dpCost = new double[nSteps, nAngles];
            int[,] dpParent = new int[nSteps, nAngles];
            int midJ = nAngles / 2;

            for (int j = 0; j < nAngles; j++)
            {
                double startBias = (mode == 1) ? Math.Pow((j - midJ) / (double)midJ, 2) * 2.0 : 0.15 * Math.Pow((j - midJ) / (double)midJ, 2);
                dpCost[0, j] = cSpaceEnergy[0, j] + startBias;
                dpParent[0, j] = j;
            }

            for (int i = 1; i < nSteps; i++)
            {
                for (int j = 0; j < nAngles; j++)
                {
                    double bestPrev = double.MaxValue;
                    int bestP = j;

                    for (int k = 0; k < nAngles; k++)
                    {
                        // Riemannian transition metric: penalizes drastic spindle rotation + path jump
                        double dAng = (candZAxes[i, j] - candZAxes[i - 1, k]).Length;
                        double dGrid = Math.Abs(j - k) / (double)nAngles;
                        double elasticStep = 3.2 * dAng * dAng + 4.5 * dGrid * dGrid;

                        if (mode == 1)
                        {
                            double dPos = candOrigins[i, j].DistanceTo(candOrigins[i - 1, k]) / Math.Max(1e-4, diag / nSteps);
                            elasticStep += 0.45 * dPos * dPos;
                        }

                        double candidateTotal = dpCost[i - 1, k] + elasticStep + cSpaceEnergy[i, j] + 0.08 * Math.Pow((j - midJ) / (double)midJ, 2);
                        if (candidateTotal < bestPrev)
                        {
                            bestPrev = candidateTotal;
                            bestP = k;
                        }
                    }

                    dpCost[i, j] = bestPrev;
                    dpParent[i, j] = bestP;
                }
            }

            // Backtrack optimal geodesic index sequence j*(i) from B to A
            int[] optJ = new int[nSteps];
            double minEndCost = double.MaxValue;
            int bestEndJ = midJ;
            for (int j = 0; j < nAngles; j++)
            {
                double endBias = (mode == 1) ? Math.Pow((j - midJ) / (double)midJ, 2) * 2.0 : 0.15 * Math.Pow((j - midJ) / (double)midJ, 2);
                if (dpCost[nSteps - 1, j] + endBias < minEndCost)
                {
                    minEndCost = dpCost[nSteps - 1, j] + endBias;
                    bestEndJ = j;
                }
            }

            optJ[nSteps - 1] = bestEndJ;
            for (int i = nSteps - 1; i >= 1; i--)
                optJ[i - 1] = dpParent[i, optJ[i]];

            // 5. Continuous Repulsive Shell Smoothing (Laplacian Sobolev Preconditioned Flow on Positions & SO(3) Frames)
            Point3d[] optOrigins = new Point3d[nSteps];
            Vector3d[] optZ = new Vector3d[nSteps];
            double[] smoothJ = new double[nSteps];
            clearanceMarginPerStep = new double[nSteps];

            for (int i = 0; i < nSteps; i++)
            {
                int j = optJ[i];
                optOrigins[i] = candOrigins[i, j];
                optZ[i] = candZAxes[i, j];
                smoothJ[i] = j;
                clearanceMarginPerStep[i] = cSpaceClearance[i, j];
            }

            // Smooth the geodesic trajectory & spindle axes while pushing away from obstacles (Repulsive Curves/Shells flow)
            for (int iter = 0; iter < 12; iter++)
            {
                Point3d[] nextOrig = (Point3d[])optOrigins.Clone();
                Vector3d[] nextZ = (Vector3d[])optZ.Clone();
                double[] nextJ = (double[])smoothJ.Clone();

                for (int i = 1; i < nSteps - 1; i++)
                {
                    // Smooth C-Space path coordinate
                    nextJ[i] = 0.25 * smoothJ[i - 1] + 0.50 * smoothJ[i] + 0.25 * smoothJ[i + 1];

                    // Smooth SO(3) spindle Z-axis
                    Vector3d zAvg = 0.25 * optZ[i - 1] + 0.50 * optZ[i] + 0.25 * optZ[i + 1];
                    if (zAvg.Length > 1e-10) zAvg.Unitize();
                    else zAvg = optZ[i];

                    Point3d pAvg = optOrigins[i];
                    if (mode == 1)
                    {
                        // Smooth 3D Pick & Place curve while applying repulsive obstacle push
                        pAvg = 0.25 * optOrigins[i - 1] + 0.50 * optOrigins[i] + 0.25 * optOrigins[i + 1];
                        foreach (Point3d op in obsPts)
                        {
                            Vector3d diff = pAvg - op;
                            double d = diff.Length;
                            if (d < 1.8 * rSpindle && d > 1e-6)
                            {
                                diff.Unitize();
                                pAvg += diff * (0.18 * (1.8 * rSpindle - d));
                            }
                        }
                    }

                    // Keep whichever orientation has lower repulsive shell energy
                    double eSmooth = EvaluateSpindleShellEnergy(pAvg, zAvg, out double clrSmooth);
                    double eCurr = EvaluateSpindleShellEnergy(optOrigins[i], optZ[i], out double clrCurr);
                    if (eSmooth <= eCurr + 0.35)
                    {
                        nextOrig[i] = pAvg;
                        nextZ[i] = zAvg;
                        clearanceMarginPerStep[i] = clrSmooth;
                    }
                }

                optOrigins = nextOrig;
                optZ = nextZ;
                smoothJ = nextJ;
            }

            // Reconstruct parallel-transported Robot Planes along the optimal trajectory
            Vector3d prevX = baseFrames[0].XAxis - (baseFrames[0].XAxis * optZ[0]) * optZ[0];
            if (prevX.Length < 1e-10)
                prevX = Vector3d.CrossProduct(optZ[0], Math.Abs(optZ[0].X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
            prevX.Unitize();

            for (int i = 0; i < nSteps; i++)
            {
                Vector3d z = optZ[i];
                Vector3d x = prevX - (prevX * z) * z;
                if (x.Length < 1e-10)
                    x = Vector3d.CrossProduct(z, Math.Abs(z.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                x.Unitize();
                prevX = x;
                Vector3d y = Vector3d.CrossProduct(z, x);
                y.Unitize();
                optimalRobotPlanes.Add(new Plane(optOrigins[i], x, y));
            }

            // 6. Construct the 2D Configuration-Space (C-Space) Manifold Mesh & Geodesic Path Curve for Viewport Analysis
            Point3d mapOrigin = new Point3d(sceneBox.Max.X + 0.28 * diag, sceneBox.Min.Y, sceneBox.Min.Z);
            double mapWidth = 0.85 * diag;
            double mapDepth = 0.52 * diag;
            double mapMaxHeight = 0.22 * diag;

            double maxVisE = 1e-6;
            for (int i = 0; i < nSteps; i++)
                for (int j = 0; j < nAngles; j++)
                    if (cSpaceEnergy[i, j] > maxVisE) maxVisE = cSpaceEnergy[i, j];
            maxVisE = Math.Min(maxVisE, 25.0);

            for (int i = 0; i < nSteps; i++)
            {
                double u = i / (double)(nSteps - 1);
                for (int j = 0; j < nAngles; j++)
                {
                    double v = j / (double)(nAngles - 1);
                    double normE = Math.Min(1.0, Math.Log(1.0 + cSpaceEnergy[i, j]) / Math.Log(1.0 + maxVisE));
                    double zHeight = normE * mapMaxHeight;

                    Point3d ptMap = mapOrigin + new Vector3d(u * mapWidth, v * mapDepth, zHeight);
                    cSpaceMapMesh.Vertices.Add(ptMap);
                    // Collision & Singularity Heatmap: Deep Blue/Teal (0 = safe valley) -> Gold (0.5) -> Bright Red (1 = collision/singularity peak)
                    int rCol = (int)Math.Max(0, Math.Min(255, normE < 0.5 ? (30 + normE * 2.0 * 215) : 245));
                    int gCol = (int)Math.Max(0, Math.Min(255, normE < 0.5 ? (145 + normE * 2.0 * 75) : (220 - (normE - 0.5) * 2.0 * 185)));
                    int bCol = (int)Math.Max(0, Math.Min(255, normE < 0.5 ? (220 - normE * 2.0 * 160) : (60 - (normE - 0.5) * 2.0 * 45)));
                    cSpaceMapMesh.VertexColors.Add(Color.FromArgb(rCol, gCol, bCol));
                }
            }

            for (int i = 0; i < nSteps - 1; i++)
            {
                for (int j = 0; j < nAngles - 1; j++)
                {
                    int v00 = i * nAngles + j;
                    int v10 = (i + 1) * nAngles + j;
                    int v11 = (i + 1) * nAngles + (j + 1);
                    int v01 = i * nAngles + (j + 1);
                    cSpaceMapMesh.Faces.AddFace(v00, v10, v11, v01);
                }
            }
            cSpaceMapMesh.Normals.ComputeNormals();

            // Extract the 2D Geodesic Path Polyline across the C-Space Map Mesh
            for (int i = 0; i < nSteps; i++)
            {
                double u = i / (double)(nSteps - 1);
                double v = Math.Max(0.0, Math.Min(1.0, smoothJ[i] / (nAngles - 1.0)));
                int jIdx = Math.Max(0, Math.Min(nAngles - 1, (int)Math.Round(smoothJ[i])));
                double normE = Math.Min(1.0, Math.Log(1.0 + cSpaceEnergy[i, jIdx]) / Math.Log(1.0 + maxVisE));
                double zHeight = normE * mapMaxHeight + 0.008 * diag; // slightly lifted above mesh for crisp visibility
                cSpaceGeodesicPath.Add(mapOrigin + new Vector3d(u * mapWidth, v * mapDepth, zHeight));
            }

            // 7. Build the 3D Spindle & Tool-Holder Shell Preview Mesh along the Trajectory
            int shellStride = Math.Max(1, nSteps / 11);
            int nSides = 12;
            for (int i = 0; i < nSteps; i += shellStride)
            {
                Plane pl = optimalRobotPlanes[i];
                Point3d tip = pl.Origin;
                Point3d coneTop = tip + pl.ZAxis * (1.25 * rSpindle);
                Point3d bodyTop = tip + pl.ZAxis * spindleLen;

                int baseV = spindleShellPreviewMesh.Vertices.Count;
                spindleShellPreviewMesh.Vertices.Add(tip); // baseV + 0

                for (int k = 0; k < nSides; k++)
                {
                    double ang = (2.0 * Math.PI * k) / nSides;
                    Vector3d radial = Math.Cos(ang) * pl.XAxis + Math.Sin(ang) * pl.YAxis;
                    spindleShellPreviewMesh.Vertices.Add(coneTop + radial * (0.85 * rSpindle)); // ring 1: baseV + 1 + k
                    spindleShellPreviewMesh.Vertices.Add(bodyTop + radial * rSpindle);          // ring 2: baseV + 1 + 2k + 1
                }

                Color cShell = cSpaceEnergy[i, optJ[i]] < 1.0
                    ? Color.FromArgb(140, 60, 220, 180)
                    : Color.FromArgb(160, 255, 140, 60);

                for (int v = baseV; v < spindleShellPreviewMesh.Vertices.Count; v++)
                    spindleShellPreviewMesh.VertexColors.Add(cShell);

                for (int k = 0; k < nSides; k++)
                {
                    int kNext = (k + 1) % nSides;
                    int r1_k = baseV + 1 + 2 * k;
                    int r2_k = baseV + 2 + 2 * k;
                    int r1_n = baseV + 1 + 2 * kNext;
                    int r2_n = baseV + 2 + 2 * kNext;

                    // Nozzle cone face
                    spindleShellPreviewMesh.Faces.AddFace(baseV, r1_k, r1_n);
                    // Spindle cylinder quad
                    spindleShellPreviewMesh.Faces.AddFace(r1_k, r2_k, r2_n, r1_n);
                }
            }
            spindleShellPreviewMesh.Normals.ComputeNormals();

            return new Polyline(optOrigins);
        }

        // =========================================================================================
        // 28. MULTI-MODE VECTOR, 4-ROSY CROSS, GEODESIC GRADIENT & COVARIANT VECTOR GRADIENT VIEWER
        //     Computes Discrete Divergence (div V), Rotor/Curl ((curl V).N), Covariant Vector
        //     Gradient (nabla ||V|| + (V.nabla)V), and 4-RoSy Cross Glyphs.
        // =========================================================================================
        public static void ComputeVectorFieldGradientsAndGlyphs(
            DdgMesh mesh,
            IList<Vector3d> optionalVectors,
            IList<double> optionalScalars,
            double scaleMultiplier,
            int displayMode,
            out List<Line> primaryLines,
            out List<Line> crossOrOrthoLines,
            out Vector3d[] covariantVectorGradient,
            out double[] divergenceField,
            out double[] curlField,
            out Mesh coloredAnalysisMesh,
            out List<Color> primaryColors)
        {
            int nV = mesh.VertexCount;
            primaryLines = new List<Line>(nV);
            crossOrOrthoLines = new List<Line>(nV);
            covariantVectorGradient = new Vector3d[nV];
            divergenceField = new double[nV];
            curlField = new double[nV];
            primaryColors = new List<Color>(nV);

            // 1. Obtain vertex tangent vector field V (from Vectors, Scalar Geodesic Gradient, or Principal Curvature)
            Vector3d[] vField;
            double[] magField = new double[nV];

            if (optionalVectors != null && optionalVectors.Count > 0)
            {
                vField = mesh.MapVectorInputToVertices(optionalVectors);
            }
            else if (optionalScalars != null && optionalScalars.Count > 0)
            {
                double[] u = mesh.MapScalarInputToVertices(optionalScalars);
                mesh.ComputeGradient(u, out _, out vField);
            }
            else
            {
                vField = ComputeOptimalDirectionField(mesh, 4, 0.65, out _, out _);
            }

            // Project onto tangent plane and compute magnitudes
            double maxMag = 1e-12;
            for (int i = 0; i < nV; i++)
            {
                Vector3d n = mesh.VertexNormals[i];
                Vector3d v = vField[i] - (vField[i] * n) * n;
                double len = v.Length;
                magField[i] = len;
                if (len > maxMag) maxMag = len;
                vField[i] = v;
            }

            // 2. Compute Discrete Divergence (div V), Curl ((nabla x V) . N), and Covariant Vector Gradient
            //    First compute face vectors and face gradient of ||V||
            var faceVecs = new Vector3d[mesh.FaceCount];
            for (int f = 0; f < mesh.FaceCount; f++)
            {
                int[] fv = mesh.Faces[f];
                Vector3d vf = (vField[fv[0]] + vField[fv[1]] + vField[fv[2]]) / 3.0;
                Vector3d nf = mesh.FaceNormals[f];
                vf -= (vf * nf) * nf;
                faceVecs[f] = vf;
            }

            divergenceField = mesh.ComputeIntegratedDivergence(faceVecs);

            // Compute discrete Curl by evaluating divergence of 90-degree rotated face field (N x V)
            var rotatedFaceVecs = new Vector3d[mesh.FaceCount];
            for (int f = 0; f < mesh.FaceCount; f++)
            {
                rotatedFaceVecs[f] = Vector3d.CrossProduct(mesh.FaceNormals[f], faceVecs[f]);
            }
            curlField = mesh.ComputeIntegratedDivergence(rotatedFaceVecs);

            // Normalize Div and Curl by dual area for scale-invariant inspection
            for (int i = 0; i < nV; i++)
            {
                double area = Math.Max(1e-8, mesh.DualAreas[i]);
                divergenceField[i] /= area;
                curlField[i] /= area;
            }

            // Compute Covariant Vector Gradient: combines gradient of magnitude nabla(||V||) + spatial turning rate (V . nabla)V
            mesh.ComputeGradient(magField, out _, out Vector3d[] gradMag);
            for (int i = 0; i < nV; i++)
            {
                Vector3d n = mesh.VertexNormals[i];
                Vector3d turnVec = Vector3d.Zero;
                Point3d pi = mesh.Positions[i];
                Vector3d vi = vField[i];
                var outHes = mesh.VertexOutgoingHalfEdges[i];

                foreach (int heIdx in outHes)
                {
                    int nb = mesh.HalfEdges[heIdx].TargetVertex;
                    Vector3d edge = mesh.Positions[nb] - pi;
                    double eLen2 = edge.SquareLength;
                    if (eLen2 < 1e-12) continue;
                    // Parallel transport neighbor vector v_nb to tangent plane at i
                    Vector3d vNb = vField[nb] - (vField[nb] * n) * n;
                    Vector3d dV = vNb - vi;
                    turnVec += (dV.Length / Math.Sqrt(eLen2)) * edge;
                }
                if (outHes.Count > 0)
                    turnVec /= outHes.Count;

                Vector3d covGrad = gradMag[i] + 0.5 * (turnVec - (turnVec * n) * n);
                covariantVectorGradient[i] = covGrad;
            }

            // 3. Choose scalar field to color the analysis mesh based on displayMode:
            //    0 = Vector Arrows (colored by Magnitude ||V||)
            //    1 = 4-RoSy Symmetric Crosses (colored by Curl / Anisotropy)
            //    2 = Geodesic Gradient & Wavefront Ticks (colored by Gradient Slope ||nabla u||)
            //    3 = Covariant Vector Gradient & Divergence-Curl Jacobian (colored by ||nabla V||)
            double[] colorScalars = new double[nV];
            double minS = double.MaxValue, maxS = double.MinValue;
            for (int i = 0; i < nV; i++)
            {
                double val = displayMode == 1 ? Math.Abs(curlField[i])
                           : displayMode == 3 ? covariantVectorGradient[i].Length
                           : magField[i];
                colorScalars[i] = val;
                if (val < minS) minS = val;
                if (val > maxS) maxS = val;
            }
            double spanS = Math.Max(1e-12, maxS - minS);
            coloredAnalysisMesh = ColorMeshByPalette(mesh.RhinoMesh, colorScalars, displayMode == 3 ? 3 : 2);

            // 4. Generate 3D Glyph Lines (Arrows, 4-RoSy Crosses, Wavefront Ticks, or Covariant Gradient Vectors)
            double baseLen = Math.Max(1e-5, scaleMultiplier * mesh.MeanEdgeLength);

            for (int i = 0; i < nV; i++)
            {
                Point3d p = mesh.Positions[i];
                Vector3d n = mesh.VertexNormals[i];
                Vector3d dir = vField[i];
                if (dir.Length < 1e-12) continue;
                dir.Unitize();

                Vector3d ortho = Vector3d.CrossProduct(n, dir);
                if (ortho.Length > 1e-12) ortho.Unitize();

                double tCol = (colorScalars[i] - minS) / spanS;
                Color cGlyph = EvaluateScientificColor(tCol, displayMode == 3 ? 3 : 2);

                if (displayMode == 1)
                {
                    // Mode 1: 4-RoSy Symmetric Crosses (U and V orthogonal axes centered at vertex)
                    double half = 0.55 * baseLen;
                    primaryLines.Add(new Line(p - half * dir, p + half * dir));
                    crossOrOrthoLines.Add(new Line(p - half * ortho, p + half * ortho));
                    primaryColors.Add(cGlyph);
                }
                else if (displayMode == 2)
                {
                    // Mode 2: Geodesic Gradient Arrow + Orthogonal Iso-Wavefront Tick
                    primaryLines.Add(new Line(p, p + baseLen * dir));
                    double tickHalf = 0.38 * baseLen;
                    crossOrOrthoLines.Add(new Line(p - tickHalf * ortho, p + tickHalf * ortho));
                    primaryColors.Add(cGlyph);
                }
                else if (displayMode == 3)
                {
                    // Mode 3: Vector Gradient (Covariant Derivative Arrow + Primary Vector Cross)
                    Vector3d vg = covariantVectorGradient[i];
                    if (vg.Length > 1e-12) vg.Unitize();
                    else vg = ortho;
                    primaryLines.Add(new Line(p, p + baseLen * vg));
                    double half = 0.42 * baseLen;
                    crossOrOrthoLines.Add(new Line(p - half * dir, p + half * dir));
                    primaryColors.Add(cGlyph);
                }
                else
                {
                    // Mode 0 (Default): Directed Tangent Vector Arrows + Orthogonal Cross Output
                    double scaleByMag = (0.45 + 0.55 * (magField[i] / maxMag)) * baseLen;
                    primaryLines.Add(new Line(p, p + scaleByMag * dir));
                    double half = 0.35 * baseLen;
                    crossOrOrthoLines.Add(new Line(p - half * ortho, p + half * ortho));
                    primaryColors.Add(cGlyph);
                }
            }
        }

        // =========================================================================================
        // 29. SCALAR-WEIGHTED + VECTOR-GUIDED ANISOTROPIC FLOW & GEODESIC TOOLPATH ("GuidedFlow")
        //     Inspired by Variable-Coefficient Drift-Diffusion (Sawhney et al. / Crane),
        //     Anisotropic Geodesics in Heat, and Variational Surface Cutting.
        //     - Scalar Relevance W in [0, 1]: 0 = Forbidden / Avoid, 1 = High-Priority Highway / Attract
        //     - Vector Field V: Desired travel direction on the mesh
        //     - Combines V with the Relevance Gradient (+nabla W) so trajectories are actively pushed
        //       out of 0-zones into 1-zones while traveling in the direction of V!
        // =========================================================================================
        public static List<Polyline> ComputeScalarWeightedVectorGuidedToolpaths(
            DdgMesh mesh,
            IList<Vector3d> optionalVectorField,
            IList<double> rawRelevanceScalars,
            IList<Point3d> optionalSeedOrWaypoints,
            out Polyline anisotropicABGeodesic,
            out Vector3d[] guidedDriftField,
            out List<Plane> fabricationPlanes,
            out Mesh relevanceColoredMesh,
            out double[] normalizedRelevance)
        {
            int nV = mesh.VertexCount;
            anisotropicABGeodesic = new Polyline();
            guidedDriftField = new Vector3d[nV];
            fabricationPlanes = new List<Plane>();
            normalizedRelevance = new double[nV];

            // 1. Normalize Scalar Relevance Field W to [0, 1] (0 = do not pass, 1 = high importance highway)
            if (rawRelevanceScalars != null && rawRelevanceScalars.Count > 0)
            {
                double[] rawW = mesh.MapScalarInputToVertices(rawRelevanceScalars);
                double minW = double.MaxValue, maxW = double.MinValue;
                for (int i = 0; i < nV; i++)
                {
                    if (rawW[i] < minW) minW = rawW[i];
                    if (rawW[i] > maxW) maxW = rawW[i];
                }
                double spanW = maxW - minW;
                for (int i = 0; i < nV; i++)
                {
                    normalizedRelevance[i] = spanW > 1e-12
                        ? Math.Max(0.0, Math.Min(1.0, (rawW[i] - minW) / spanW))
                        : 1.0;
                }
            }
            else
            {
                // Default if no scalar provided: use normalized Mean Curvature magnitude as relevance
                double[] hCurv = mesh.MeanCurvature;
                double minH = double.MaxValue, maxH = double.MinValue;
                for (int i = 0; i < nV; i++)
                {
                    double v = Math.Abs(hCurv[i]);
                    if (v < minH) minH = v;
                    if (v > maxH) maxH = v;
                }
                double spanH = Math.Max(1e-12, maxH - minH);
                for (int i = 0; i < nV; i++)
                    normalizedRelevance[i] = (Math.Abs(hCurv[i]) - minH) / spanH;
            }

            // Smooth relevance slightly to get a clean, long-range Attractive Gradient (+nabla W) that pulls paths from 0 -> 1
            double[] smoothW = SmoothScalarField(mesh, normalizedRelevance, 2.0, 0.0, out _, out _);
            mesh.ComputeGradient(smoothW, out _, out Vector3d[] gradW);

            // 2. Obtain base guidance Vector Field V
            Vector3d[] baseV;
            if (optionalVectorField != null && optionalVectorField.Count > 0)
            {
                baseV = mesh.MapVectorInputToVertices(optionalVectorField);
            }
            else
            {
                baseV = ComputeOptimalDirectionField(mesh, 1, 0.65, out _, out _);
            }

            // 3. Construct the Relevance-Guided Drift-Advection Vector Field:
            //    U(x) = W(x)^0.65 * V_unit(x) + lambda * (1.15 - 0.65 * W(x)) * gradW_unit(x)
            //    - In low-W zones (W -> 0), the gradient +nabla W strongly dominates and pushes the trajectory UPHILL toward W = 1!
            //    - In high-W zones (W -> 1), the vector field V dominates so the trajectory surfs cleanly along V inside the 1-corridor!
            for (int i = 0; i < nV; i++)
            {
                Vector3d n = mesh.VertexNormals[i];
                Vector3d v = baseV[i] - (baseV[i] * n) * n;
                if (v.Length > 1e-12) v.Unitize();

                Vector3d gw = gradW[i] - (gradW[i] * n) * n;
                if (gw.Length > 1e-12) gw.Unitize();

                double w = normalizedRelevance[i];
                // Ensure the vector field V doesn't point steeply downhill into a 0-zone if we can flip or deflect it
                double dotDownhill = v * gw;
                Vector3d vDeflected = v;
                if (w < 0.35 && dotDownhill < -0.1)
                {
                    // Project out the component pointing into the forbidden 0-zone
                    vDeflected = v - (dotDownhill * gw);
                    if (vDeflected.Length > 1e-12) vDeflected.Unitize();
                }

                double wVecWeight = 0.25 + 0.75 * Math.Pow(w, 0.65);
                double wGradPush = 0.62 * (1.15 - 0.75 * w);

                Vector3d combined = wVecWeight * vDeflected + wGradPush * gw;
                combined -= (combined * n) * n;
                if (combined.Length > 1e-12) combined.Unitize();
                else combined = v;

                // Scale magnitude by (0.15 + 0.85 * w) so visual arrows clearly show relevance strength
                guidedDriftField[i] = combined * (0.20 + 0.80 * w);
            }

            // Color mesh with Turbo / Heat palette: 0 (Deep Blue = avoid) -> 1 (Gold/Red = high priority highway)
            relevanceColoredMesh = ColorMeshByPalette(mesh.RhinoMesh, normalizedRelevance, 3);

            // 4. Trace Relevance-Guided Vector Toolpaths (Streamlines pushed toward W=1 and repelled from W=0)
            var seedPoints = new List<Point3d>();
            if (optionalSeedOrWaypoints != null && optionalSeedOrWaypoints.Count > 0)
            {
                seedPoints.AddRange(optionalSeedOrWaypoints);
            }
            else
            {
                // Auto-seed in high-relevance (W >= 0.45) vertices well-spaced across the mesh
                int targetSeeds = Math.Min(18, Math.Max(4, nV / 45));
                var chosen = new List<int>();
                double minSeedDist = mesh.MeanEdgeLength * 3.0;

                // Sort vertices by descending relevance W
                int[] order = new int[nV];
                for (int i = 0; i < nV; i++) order[i] = i;
                double[] relCopy = normalizedRelevance;
                Array.Sort(order, (a, b) => relCopy[b].CompareTo(relCopy[a]));

                for (int idx = 0; idx < nV && chosen.Count < targetSeeds; idx++)
                {
                    int v = order[idx];
                    if (normalizedRelevance[v] < 0.20) break;
                    bool farEnough = true;
                    foreach (int c in chosen)
                    {
                        if (mesh.Positions[v].DistanceTo(mesh.Positions[c]) < minSeedDist)
                        {
                            farEnough = false;
                            break;
                        }
                    }
                    if (farEnough)
                    {
                        chosen.Add(v);
                        seedPoints.Add(mesh.Positions[v]);
                    }
                }
            }

            var flowPaths = new List<Polyline>();
            double stepSize = 0.45 * mesh.MeanEdgeLength;
            int maxSteps = Math.Min(260, Math.Max(60, (int)(mesh.RhinoMesh.GetBoundingBox(false).Diagonal.Length / Math.Max(1e-5, stepSize) * 1.4)));

            foreach (Point3d seed in seedPoints)
            {
                var pts = new List<Point3d>();
                MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(seed, 0.0);
                if (mp == null) continue;

                Point3d curr = mp.Point;
                pts.Add(curr);
                Vector3d prevDir = Vector3d.Zero;

                for (int step = 0; step < maxSteps; step++)
                {
                    MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                    double wCurr = mp.T[0] * normalizedRelevance[mf.A]
                                 + mp.T[1] * normalizedRelevance[mf.B]
                                 + mp.T[2] * normalizedRelevance[mf.C];

                    // Stop if trajectory hits a hard forbidden 0-barrier (after leaving initial seed)
                    if (step > 4 && wCurr < 0.04) break;

                    Vector3d dir = mp.T[0] * guidedDriftField[mf.A]
                                 + mp.T[1] * guidedDriftField[mf.B]
                                 + mp.T[2] * guidedDriftField[mf.C];

                    if (dir.Length < 1e-10) break;
                    dir.Unitize();

                    // Prevent sudden 180-degree reversals
                    if (prevDir.Length > 1e-10 && (dir * prevDir) < -0.5) break;
                    prevDir = dir;

                    Point3d nextCand = curr + stepSize * dir;
                    MeshPoint nextMp = mesh.RhinoMesh.ClosestMeshPoint(nextCand, 0.0);
                    if (nextMp == null) break;

                    if (nextMp.Point.DistanceTo(curr) < 0.05 * stepSize) break;
                    curr = nextMp.Point;
                    mp = nextMp;
                    pts.Add(curr);
                }

                if (pts.Count >= 2)
                    flowPaths.Add(new Polyline(pts));
            }

            // 5. Compute the Optimal Anisotropic Scalar+Vector Geodesic Toolpath from A to B (across waypoints in P, or between top 2 seeds)
            //    Uses a Randers / Weighted Riemannian edge cost:
            //    Cost(i -> j) = ||p_j - p_i|| / (0.04 + W_avg^2.2) * (1.0 - 0.58 * cos(theta_V))
            var waypoints = (optionalSeedOrWaypoints != null && optionalSeedOrWaypoints.Count >= 2)
                ? optionalSeedOrWaypoints
                : seedPoints;

            if (waypoints.Count >= 2)
            {
                var fullAB = new List<Point3d>();
                for (int wp = 0; wp < waypoints.Count - 1; wp++)
                {
                    int vStart = mesh.FindClosestVertex(waypoints[wp]);
                    int vEnd = mesh.FindClosestVertex(waypoints[wp + 1]);
                    if (vStart == vEnd) continue;

                    double[] dist = new double[nV];
                    int[] parent = new int[nV];
                    bool[] visited = new bool[nV];
                    for (int i = 0; i < nV; i++)
                    {
                        dist[i] = double.MaxValue;
                        parent[i] = -1;
                    }
                    dist[vStart] = 0.0;

                    var pq = new SortedSet<(double cost, int v)>();
                    pq.Add((0.0, vStart));

                    while (pq.Count > 0)
                    {
                        var top = pq.Min;
                        pq.Remove(top);
                        int u = top.v;
                        if (visited[u]) continue;
                        visited[u] = true;
                        if (u == vEnd) break;

                        Point3d pu = mesh.Positions[u];
                        Vector3d vu = guidedDriftField[u];
                        if (vu.Length > 1e-12) vu.Unitize();

                        foreach (int heIdx in mesh.VertexOutgoingHalfEdges[u])
                        {
                            int nb = mesh.HalfEdges[heIdx].TargetVertex;
                            if (visited[nb]) continue;
                            Vector3d edge = mesh.Positions[nb] - pu;
                            double eLen = edge.Length;
                            if (eLen < 1e-12) continue;
                            Vector3d eDir = edge / eLen;

                            // 1) Scalar Relevance Weight: W -> 1 is 25x cheaper ("highway"), W -> 0 is 25x more expensive ("barrier")
                            double wAvg = 0.5 * (normalizedRelevance[u] + normalizedRelevance[nb]);
                            double scalarPenalty = 1.0 / (0.04 + 2.4 * wAvg * wAvg);

                            // 2) Vector Field Alignment Drift: moving WITH the vector field V is strongly favored
                            double alignCos = vu * eDir; // in [-1, 1]
                            double vectorFactor = 1.0 - 0.58 * alignCos; // 0.42 when aligned (+V), 1.58 when opposing (-V)

                            double stepCost = eLen * scalarPenalty * vectorFactor;
                            double newCost = dist[u] + stepCost;
                            if (newCost < dist[nb])
                            {
                                dist[nb] = newCost;
                                parent[nb] = u;
                                pq.Add((newCost, nb));
                            }
                        }
                    }

                    // Reconstruct and smooth segment on the mesh
                    var segVerts = new List<int>();
                    int currV = vEnd;
                    while (currV != -1)
                    {
                        segVerts.Add(currV);
                        if (currV == vStart) break;
                        currV = parent[currV];
                    }
                    segVerts.Reverse();

                    if (segVerts.Count >= 2)
                    {
                        var segPts = new Point3d[segVerts.Count];
                        for (int k = 0; k < segVerts.Count; k++)
                            segPts[k] = mesh.Positions[segVerts[k]];

                        // Intrinsic laplacian smoothing projected onto mesh while biasing toward higher W
                        for (int pass = 0; pass < 6; pass++)
                        {
                            for (int k = 1; k < segPts.Length - 1; k++)
                            {
                                Point3d avg = 0.25 * segPts[k - 1] + 0.50 * segPts[k] + 0.25 * segPts[k + 1];
                                MeshPoint mpS = mesh.RhinoMesh.ClosestMeshPoint(avg, 0.0);
                                if (mpS != null) segPts[k] = mpS.Point;
                            }
                        }

                        for (int k = (wp == 0 ? 0 : 1); k < segPts.Length; k++)
                            fullAB.Add(segPts[k]);
                    }
                }

                if (fullAB.Count >= 2)
                    anisotropicABGeodesic = new Polyline(fullAB);
            }

            // 6. Generate 5/6-Axis Fabrication Planes along the primary path(s)
            IList<Polyline> targetPathsForPlanes = anisotropicABGeodesic.Count >= 2
                ? new List<Polyline> { anisotropicABGeodesic }
                : flowPaths;

            foreach (Polyline pl in targetPathsForPlanes)
            {
                if (pl == null || pl.Count < 2) continue;
                for (int i = 0; i < pl.Count; i++)
                {
                    Point3d p = pl[i];
                    Vector3d tang = (i < pl.Count - 1) ? (pl[i + 1] - p) : (p - pl[i - 1]);
                    if (tang.Length < 1e-10) tang = Vector3d.XAxis;
                    tang.Unitize();

                    int vNear = mesh.FindClosestVertex(p);
                    Vector3d nrm = mesh.VertexNormals[vNear];
                    if (nrm.Length < 1e-10) nrm = Vector3d.ZAxis;
                    nrm.Unitize();

                    Vector3d yAx = Vector3d.CrossProduct(nrm, tang);
                    if (yAx.Length < 1e-10)
                        yAx = Vector3d.CrossProduct(nrm, Math.Abs(nrm.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                    yAx.Unitize();
                    Vector3d xAx = Vector3d.CrossProduct(yAx, nrm);
                    xAx.Unitize();

                    fabricationPlanes.Add(new Plane(p, xAx, yAx));
                }
            }

            return flowPaths;
        }

        // =========================================================================================
        // 30. CURVATURE REGIME, INFLECTION (H = 0) & MAXIMA/MINIMA WATERSHED MESH SEGMENTATION
        //     Segments a mesh at user-controlled Curvature Loci:
        //     - Mode 0: Cut at Inflection / Signed Threshold (H = T, default 0.0 -> Convex vs. Concave Pieces)
        //     - Mode 1: Cut Along Curvature Maxima (Ridges / Crestas -> Leaves Concave Valley Basins)
        //     - Mode 2: Cut Along Curvature Minima (Valleys / Surcos -> Leaves Convex Dome Lobes)
        //     - Mode 3: 3-Regime Band Split: Concave (H < -|T|) + Flat (-|T|..+|T|) + Convex (H > +|T|)
        // =========================================================================================
        public static List<Mesh> ComputeCurvatureRegimeSegmentation(
            DdgMesh mesh,
            int cutMode,
            double curvatureThreshold,
            double smoothness,
            out List<Mesh> convexPieces,
            out List<Mesh> concavePieces,
            out List<Polyline> subdivisionCurves,
            out double[] normalizedSignedCurvature,
            out Mesh coloredCurvatureMesh)
        {
            int nV = mesh.VertexCount;
            int nF = mesh.FaceCount;
            convexPieces = new List<Mesh>();
            concavePieces = new List<Mesh>();
            subdivisionCurves = new List<Polyline>();
            normalizedSignedCurvature = new double[nV];
            var allPieces = new List<Mesh>();

            // 1. Compute Signed Mean Curvature H at every vertex:
            //    Positive (+H) = Convex (surface bends away from outward normal)
            //    Negative (-H) = Concave (surface bends toward outward normal)
            //    Zero (0)      = Inflection / Flat / Saddle transition
            double[] rawH = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                Point3d pi = mesh.Positions[i];
                Vector3d ni = mesh.VertexNormals[i];
                var outHes = mesh.VertexOutgoingHalfEdges[i];
                if (outHes.Count == 0) continue;

                Point3d bary = Point3d.Origin;
                foreach (int heIdx in outHes)
                    bary += mesh.Positions[mesh.HalfEdges[heIdx].TargetVertex];
                bary /= outHes.Count;

                // Vector from neighbor centroid to vertex pi dotted with outward normal ni:
                // > 0 on convex peaks/ridges, < 0 in concave valleys/bowls
                Vector3d diff = pi - bary;
                rawH[i] = (diff * ni) / Math.Max(1e-6, mesh.MeanEdgeLength);
            }

            // Smooth signed curvature via Cotan-Laplace diffusion so cut contours are clean fabrication seams
            double[] smoothH = SmoothScalarField(mesh, rawH, Math.Max(0.2, smoothness), 0.0, out _, out _);

            // Normalize symmetrically by robust 96th-percentile magnitude so 0.0 is strictly preserved as H = 0!
            double[] absVals = new double[nV];
            for (int i = 0; i < nV; i++) absVals[i] = Math.Abs(smoothH[i]);
            Array.Sort(absVals);
            double scaleH = Math.Max(1e-8, absVals[(int)Math.Min(nV - 1, Math.Floor(nV * 0.96))]);

            for (int i = 0; i < nV; i++)
            {
                normalizedSignedCurvature[i] = Math.Max(-1.0, Math.Min(1.0, smoothH[i] / scaleH));
            }

            // Color inspection mesh with CoolWarm Diverging Palette (Blue = Concave -1, White = Zero Inflection 0, Red = Convex +1)
            coloredCurvatureMesh = new Mesh();
            coloredCurvatureMesh.CopyFrom(mesh.RhinoMesh);
            coloredCurvatureMesh.VertexColors.Clear();
            for (int i = 0; i < nV; i++)
            {
                double t01 = 0.5 * (normalizedSignedCurvature[i] + 1.0); // maps [-1, +1] -> [0, 1]
                coloredCurvatureMesh.VertexColors.Add(EvaluateScientificColor(t01, 1));
            }

            double thresh = Math.Max(-0.95, Math.Min(0.95, curvatureThreshold));

            if (cutMode == 0 || cutMode == 3)
            {
                // MODE 0: Exact Sub-Triangle Cut at H_norm = Threshold (default 0.0 -> Concave vs. Convex)
                // MODE 3: 3-Regime Band Cut at -|T| and +|T| (Concave, Flat, Convex)
                var cutLevels = new List<double>();
                if (cutMode == 3)
                {
                    double bandT = Math.Max(0.08, Math.Abs(thresh) > 1e-4 ? Math.Abs(thresh) : 0.22);
                    cutLevels.Add(-bandT);
                    cutLevels.Add(+bandT);
                }
                else
                {
                    cutLevels.Add(thresh);
                }

                subdivisionCurves = ComputeExactIsoCutAndBands(
                    mesh,
                    normalizedSignedCurvature,
                    cutLevels,
                    false, // use exact signed threshold in [-1, +1]
                    out _,
                    out _,
                    out List<Mesh> bands,
                    out _,
                    out _);

                // Split each band into connected components and classify each piece as Convex or Concave
                var distinctColors = GenerateDistinctPalette(16);
                int colorCounter = 0;

                for (int b = 0; b < bands.Count; b++)
                {
                    Mesh bandMesh = bands[b];
                    if (bandMesh == null || bandMesh.Faces.Count == 0) continue;

                    Mesh[] comps = bandMesh.SplitDisjointPieces();
                    if (comps == null || comps.Length == 0) comps = new[] { bandMesh };

                    foreach (Mesh comp in comps)
                    {
                        if (comp == null || comp.Faces.Count == 0) continue;
                        comp.Normals.ComputeNormals();

                        // Evaluate mean signed curvature of this piece
                        double sumCurv = 0.0;
                        int sampleStep = Math.Max(1, comp.Vertices.Count / 40);
                        int samples = 0;
                        for (int vi = 0; vi < comp.Vertices.Count; vi += sampleStep)
                        {
                            int nearV = mesh.FindClosestVertex(comp.Vertices[vi]);
                            sumCurv += normalizedSignedCurvature[nearV];
                            samples++;
                        }
                        double avgCurv = samples > 0 ? sumCurv / samples : 0.0;

                        Color pieceCol = distinctColors[colorCounter % distinctColors.Length];
                        colorCounter++;
                        comp.VertexColors.Clear();
                        for (int vi = 0; vi < comp.Vertices.Count; vi++)
                            comp.VertexColors.Add(pieceCol);

                        allPieces.Add(comp);
                        if (avgCurv >= thresh)
                            convexPieces.Add(comp);
                        else
                            concavePieces.Add(comp);
                    }
                }
            }
            else
            {
                // MODE 1: Cut Along Curvature Maxima (Ridges / Crestas) -> Leaves Concave Valley Basins
                // MODE 2: Cut Along Curvature Minima (Valleys / Surcos) -> Leaves Convex Dome Lobes
                bool cutAtMaxima = (cutMode == 1);

                // In Mode 1 (cut at Maxima), basin seeds are local MINIMA of curvature (valleys), and ridges act as high barriers!
                // In Mode 2 (cut at Minima), basin seeds are local MAXIMA of curvature (convex domes), and valleys act as barriers!
                double[] fieldForSeeds = new double[nV];
                for (int i = 0; i < nV; i++)
                {
                    fieldForSeeds[i] = cutAtMaxima ? -normalizedSignedCurvature[i] : normalizedSignedCurvature[i];
                }

                // Find well-spaced prominent local peaks of fieldForSeeds
                var candidatePeaks = new List<int>();
                for (int i = 0; i < nV; i++)
                {
                    bool isLocalPeak = true;
                    foreach (int heIdx in mesh.VertexOutgoingHalfEdges[i])
                    {
                        int nb = mesh.HalfEdges[heIdx].TargetVertex;
                        if (fieldForSeeds[nb] > fieldForSeeds[i] + 1e-6)
                        {
                            isLocalPeak = false;
                            break;
                        }
                    }
                    if (isLocalPeak && fieldForSeeds[i] >= thresh)
                        candidatePeaks.Add(i);
                }

                candidatePeaks.Sort((a, b) => fieldForSeeds[b].CompareTo(fieldForSeeds[a]));

                double minPeakSpacing = mesh.MeanEdgeLength * Math.Max(2.5, 1.5 + smoothness);
                var seeds = new List<int>();
                foreach (int pk in candidatePeaks)
                {
                    bool farEnough = true;
                    foreach (int s in seeds)
                    {
                        if (mesh.Positions[pk].DistanceTo(mesh.Positions[s]) < minPeakSpacing)
                        {
                            farEnough = false;
                            break;
                        }
                    }
                    if (farEnough)
                    {
                        seeds.Add(pk);
                        if (seeds.Count >= 18) break;
                    }
                }

                // Ensure at least 2 seeds if mesh is smooth
                if (seeds.Count < 2)
                {
                    int minIdx = 0, maxIdx = 0;
                    for (int i = 1; i < nV; i++)
                    {
                        if (fieldForSeeds[i] > fieldForSeeds[maxIdx]) maxIdx = i;
                        if (fieldForSeeds[i] < fieldForSeeds[minIdx]) minIdx = i;
                    }
                    seeds.Clear();
                    seeds.Add(maxIdx);
                    if (minIdx != maxIdx) seeds.Add(minIdx);
                }

                // Multi-Source Geodesic Watershed where crossing the target Curvature Extremum (Maxima in Mode 1, Minima in Mode 2)
                // carries a steep barrier weight so region boundaries lock directly onto the Maxima Ridges or Minima Valleys!
                int[] vLabel = new int[nV];
                double[] vDist = new double[nV];
                for (int i = 0; i < nV; i++)
                {
                    vLabel[i] = -1;
                    vDist[i] = double.MaxValue;
                }

                var pq = new SortedSet<(double cost, int v)>();
                for (int k = 0; k < seeds.Count; k++)
                {
                    int sv = seeds[k];
                    vLabel[sv] = k;
                    vDist[sv] = 0.0;
                    pq.Add((0.0, sv));
                }

                while (pq.Count > 0)
                {
                    var top = pq.Min;
                    pq.Remove(top);
                    int u = top.v;

                    foreach (int heIdx in mesh.VertexOutgoingHalfEdges[u])
                    {
                        int nb = mesh.HalfEdges[heIdx].TargetVertex;
                        double eLen = mesh.Positions[u].DistanceTo(mesh.Positions[nb]);
                        // Barrier is high where curvature approaches the cut locus (Maxima +1 in Mode 1, Minima -1 in Mode 2)
                        double locusVal = cutAtMaxima
                            ? 0.5 * (normalizedSignedCurvature[u] + normalizedSignedCurvature[nb])
                            : -0.5 * (normalizedSignedCurvature[u] + normalizedSignedCurvature[nb]);

                        double barrier = 1.0 + 18.0 * Math.Pow(Math.Max(0.0, locusVal - thresh + 0.5), 2.5);
                        double newCost = vDist[u] + eLen * barrier;
                        if (newCost < vDist[nb])
                        {
                            vDist[nb] = newCost;
                            vLabel[nb] = vLabel[u];
                            pq.Add((newCost, nb));
                        }
                    }
                }

                // Assign face labels and enforce connected regions
                int[] faceLabels = new int[nF];
                for (int f = 0; f < nF; f++)
                {
                    int[] fv = mesh.Faces[f];
                    faceLabels[f] = vLabel[fv[0]] >= 0 ? vLabel[fv[0]] : 0;
                }
                EnforceConnectedClusterRegions(mesh, faceLabels, seeds.Count);

                // Extract sub-meshes per watershed region and classify as Convex or Concave
                var distinctColors = GenerateDistinctPalette(Math.Max(4, seeds.Count));
                List<Mesh> builtSubs = BuildSubMeshesFromFaceLabels(mesh, faceLabels, seeds.Count, distinctColors, out _);

                foreach (Mesh sub in builtSubs)
                {
                    if (sub == null || sub.Faces.Count == 0) continue;
                    allPieces.Add(sub);

                    double sumC = 0.0;
                    int stepV = Math.Max(1, sub.Vertices.Count / 35);
                    int cntV = 0;
                    for (int vi = 0; vi < sub.Vertices.Count; vi += stepV)
                    {
                        int nearV = mesh.FindClosestVertex(sub.Vertices[vi]);
                        sumC += normalizedSignedCurvature[nearV];
                        cntV++;
                    }
                    double avgC = cntV > 0 ? sumC / cntV : 0.0;
                    if (avgC >= 0.0) convexPieces.Add(sub);
                    else concavePieces.Add(sub);
                }

                // Extract smooth boundary curves along the Maxima / Minima watershed seams
                List<Polyline> rawSeams = ExtractLabelBoundaryCurves(mesh, faceLabels);
                if (rawSeams.Count > 0)
                {
                    var seamCurves = rawSeams.ConvertAll(pl => (Curve)new PolylineCurve(pl));
                    subdivisionCurves = ComputeGeodesicNetworkShorten(mesh.RhinoMesh, seamCurves, 6, true, out _);
                }
            }

            return allPieces;
        }

        // =========================================================================================
        // 31. REPULSIVE SHELL FOLDING, BENDING & SELF-COLLISION-FREE MORPH FROM STATE A TO B
        //     Inspired by "Repulsive Shells" (Sassen, Schumacher, Rumpf, Wardetzky, Crane 2024)
        //     & "Repulsive Surfaces" (Yu, Schumacher, Crane 2021).
        //     Deforms or folds a shell mesh M_A toward target state M_B (or self-folding arch)
        //     using Sobolev-preconditioned Tangent-Point Self-Collision Repulsion + Elastic Membrane preservation!
        // =========================================================================================
        public static Mesh ComputeRepulsiveShellFoldingAndSelfCollision(
            DdgMesh shellA,
            Mesh optionalTargetShellB,
            IList<Point3d> optionalTargetHandles,
            double foldParamT,
            double repulsionStrength,
            out List<Mesh> intermediateShellSequence,
            out List<Polyline> vertexTrajectories,
            out double[] selfClearanceRadii,
            out List<Point3d> nearSelfContactPoints)
        {
            int nV = shellA.VertexCount;
            intermediateShellSequence = new List<Mesh>();
            vertexTrajectories = new List<Polyline>();
            selfClearanceRadii = new double[nV];
            nearSelfContactPoints = new List<Point3d>();

            BoundingBox bboxA = shellA.RhinoMesh.GetBoundingBox(false);
            double diag = Math.Max(1e-3, bboxA.Diagonal.Length);
            Point3d centerA = bboxA.Center;

            // 1. Construct Target Positions P_B for each vertex of shellA:
            //    - Case A: User provided Target Mesh M_B (same vertex count or closest-point correspondence)
            //    - Case B: User provided Target Handle Points
            //    - Case C: Automatic Self-Avoiding Origami / Shell Roll-Folding (folds the two ends of the shell inward toward each other!)
            Point3d[] posA = new Point3d[nV];
            Point3d[] posB = new Point3d[nV];
            for (int i = 0; i < nV; i++) posA[i] = shellA.Positions[i];

            if (optionalTargetShellB != null && optionalTargetShellB.Vertices.Count > 0)
            {
                if (optionalTargetShellB.Vertices.Count == nV)
                {
                    for (int i = 0; i < nV; i++)
                        posB[i] = optionalTargetShellB.Vertices.Point3dAt(i);
                }
                else
                {
                    var ddgB = new DdgMesh(optionalTargetShellB);
                    if (ddgB.VertexCount == nV)
                    {
                        for (int i = 0; i < nV; i++) posB[i] = ddgB.Positions[i];
                    }
                    else
                    {
                        for (int i = 0; i < nV; i++)
                        {
                            MeshPoint mp = optionalTargetShellB.ClosestMeshPoint(posA[i], 0.0);
                            posB[i] = mp != null ? mp.Point : posA[i];
                        }
                    }
                }
            }
            else if (optionalTargetHandles != null && optionalTargetHandles.Count >= 2)
            {
                // Smoothly deform shellA toward target handles via biharmonic / heat weights
                Vector3d avgDisp = Vector3d.Zero;
                for (int h = 0; h < optionalTargetHandles.Count; h++)
                {
                    int vNear = shellA.FindClosestVertex(optionalTargetHandles[h]);
                    avgDisp += (optionalTargetHandles[h] - posA[vNear]);
                }
                avgDisp /= optionalTargetHandles.Count;

                for (int i = 0; i < nV; i++)
                {
                    Point3d p = posA[i];
                    double uNorm = (p.X - bboxA.Min.X) / Math.Max(1e-5, bboxA.Max.X - bboxA.Min.X);
                    double foldAngle = (uNorm - 0.5) * Math.PI * 1.15;
                    Vector3d n = shellA.VertexNormals[i];
                    posB[i] = p + Math.Sin(Math.Abs(foldAngle)) * avgDisp + 0.18 * diag * Math.Cos(foldAngle) * n;
                }
            }
            else
            {
                // Automatic Cylindrical / S-Curve Shell Folding (bends the shell around its longest axis so opposite sides approach!)
                double spanX = bboxA.Max.X - bboxA.Min.X;
                double spanY = bboxA.Max.Y - bboxA.Min.Y;
                bool foldAlongX = spanX >= spanY;
                double minCoord = foldAlongX ? bboxA.Min.X : bboxA.Min.Y;
                double spanCoord = Math.Max(1e-5, foldAlongX ? spanX : spanY);
                double radiusFold = spanCoord / (1.65 * Math.PI);

                for (int i = 0; i < nV; i++)
                {
                    Point3d p = posA[i];
                    double s = ((foldAlongX ? p.X : p.Y) - minCoord) / spanCoord; // in [0, 1]
                    double theta = (s - 0.5) * (1.75 * Math.PI); // curls almost into a closed C-tube where lips approach self-contact!
                    double hOffset = p.Z - centerA.Z;
                    double rEff = Math.Max(0.1 * radiusFold, radiusFold + hOffset);

                    if (foldAlongX)
                    {
                        posB[i] = new Point3d(
                            centerA.X + rEff * Math.Sin(theta),
                            p.Y,
                            centerA.Z + radiusFold * (1.0 - Math.Cos(theta)) + hOffset * Math.Cos(theta));
                    }
                    else
                    {
                        posB[i] = new Point3d(
                            p.X,
                            centerA.Y + rEff * Math.Sin(theta),
                            centerA.Z + radiusFold * (1.0 - Math.Cos(theta)) + hOffset * Math.Cos(theta));
                    }
                }
            }

            // 2. Build 1-ring neighbor sets so adjacent vertices do not trigger self-collision repulsion
            var oneRing = new HashSet<int>[nV];
            for (int i = 0; i < nV; i++)
            {
                oneRing[i] = new HashSet<int> { i };
                foreach (int heIdx in shellA.VertexOutgoingHalfEdges[i])
                {
                    int nb = shellA.HalfEdges[heIdx].TargetVertex;
                    oneRing[i].Add(nb);
                    foreach (int he2 in shellA.VertexOutgoingHalfEdges[nb])
                        oneRing[i].Add(shellA.HalfEdges[he2].TargetVertex); // exclude 2-ring from non-local self-contact
                }
            }

            // Rest edge lengths on shellA (for membrane isometric preservation during folding)
            double[] restEdgeLen = new double[shellA.EdgeCount];
            for (int e = 0; e < shellA.EdgeCount; e++)
            {
                var ed = shellA.Edges[e];
                restEdgeLen[e] = posA[ed.V0].DistanceTo(posA[ed.V1]);
            }

            // 3. Simulate nFrames = 7 Geodesic Shell States from A (t=0) to B (t=1) with Tangent-Point Self-Collision Repulsion
            int nFrames = 7;
            var allFramePositions = new Point3d[nFrames][];
            double shellThickness = Math.Max(shellA.MeanEdgeLength * 1.45, diag * 0.045) * Math.Max(0.2, repulsionStrength * 0.45);

            // Subsample representative repulsive collider vertices for O(N) fast evaluation on dense meshes
            int repStride = Math.Max(1, nV / 320);

            for (int fIdx = 0; fIdx < nFrames; fIdx++)
            {
                double tau = fIdx / (double)(nFrames - 1);
                Point3d[] curPos = new Point3d[nV];

                // Start from previous frame blended toward target B along a curved geodesic arc
                double archLift = Math.Sin(Math.PI * tau) * 0.08 * diag;
                for (int i = 0; i < nV; i++)
                {
                    Point3d lin = (1.0 - tau) * posA[i] + tau * posB[i];
                    curPos[i] = lin + archLift * shellA.VertexNormals[i];
                }

                if (fIdx > 0 && fIdx < nFrames)
                {
                    // Iterative Sobolev-Preconditioned Tangent-Point Repulsion + Elastic Membrane Relaxation
                    int nRelax = 8;
                    for (int iter = 0; iter < nRelax; iter++)
                    {
                        var disp = new Vector3d[nV];

                        // A) Non-Local Tangent-Point Self-Collision Repulsion (Sassen et al. "Repulsive Shells")
                        if (repulsionStrength > 1e-4)
                        {
                            for (int i = 0; i < nV; i++)
                            {
                                Point3d pi = curPos[i];
                                Vector3d repForce = Vector3d.Zero;

                                for (int j = 0; j < nV; j += repStride)
                                {
                                    if (oneRing[i].Contains(j)) continue;
                                    Vector3d diff = pi - curPos[j];
                                    double d = diff.Length;
                                    if (d < shellThickness && d > 1e-8)
                                    {
                                        diff /= d;
                                        double pen = (shellThickness - d) / shellThickness;
                                        repForce += diff * (shellThickness * 0.35 * pen * pen);
                                    }
                                }
                                disp[i] += repForce;
                            }
                        }

                        // B) Elastic Membrane Edge-Length Preservation (prevents shell stretching/shrinking during folding)
                        for (int e = 0; e < shellA.EdgeCount; e++)
                        {
                            var ed = shellA.Edges[e];
                            int v0 = ed.V0, v1 = ed.V1;
                            Vector3d dVec = curPos[v1] - curPos[v0];
                            double len = dVec.Length;
                            if (len < 1e-9) continue;
                            double err = (len - restEdgeLen[e]) / len;
                            Vector3d corr = 0.18 * err * dVec;
                            disp[v0] += corr;
                            disp[v1] -= corr;
                        }

                        // C) Sobolev / Laplacian Smoothing of Displacements (mimics (I + h*Delta)^-1 fractional preconditioner!)
                        var smoothDisp = new Vector3d[nV];
                        for (int i = 0; i < nV; i++)
                        {
                            var outHes = shellA.VertexOutgoingHalfEdges[i];
                            if (outHes.Count == 0)
                            {
                                smoothDisp[i] = disp[i];
                                continue;
                            }
                            Vector3d nbAvg = Vector3d.Zero;
                            foreach (int heIdx in outHes)
                                nbAvg += disp[shellA.HalfEdges[heIdx].TargetVertex];
                            nbAvg /= outHes.Count;
                            smoothDisp[i] = 0.45 * disp[i] + 0.55 * nbAvg;
                        }

                        for (int i = 0; i < nV; i++)
                            curPos[i] += smoothDisp[i];
                    }
                }

                allFramePositions[fIdx] = curPos;

                Mesh frameMesh = shellA.RhinoMesh.DuplicateMesh();
                for (int i = 0; i < nV; i++)
                    frameMesh.Vertices.SetVertex(i, curPos[i]);
                frameMesh.Normals.ComputeNormals();
                intermediateShellSequence.Add(frameMesh);
            }

            // 4. Interpolate exact output Shell at user parameter foldParamT in [0, 1]
            double tClamped = Math.Max(0.0, Math.Min(1.0, foldParamT));
            double fReal = tClamped * (nFrames - 1);
            int f0 = Math.Min(nFrames - 2, (int)Math.Floor(fReal));
            int f1 = f0 + 1;
            double alpha = fReal - f0;

            Mesh activeShell = shellA.RhinoMesh.DuplicateMesh();
            Point3d[] activePos = new Point3d[nV];
            for (int i = 0; i < nV; i++)
            {
                activePos[i] = (1.0 - alpha) * allFramePositions[f0][i] + alpha * allFramePositions[f1][i];
                activeShell.Vertices.SetVertex(i, activePos[i]);
            }
            activeShell.Normals.ComputeNormals();

            // 5. Evaluate Tangent-Point Self-Clearance Radius & Near-Contact Stress on activeShell
            double maxClearance = diag * 0.5;
            double[] stressField = new double[nV];
            for (int i = 0; i < nV; i++)
            {
                Point3d pi = activePos[i];
                Vector3d ni = activeShell.Normals.Count > i ? (Vector3d)activeShell.Normals[i] : shellA.VertexNormals[i];
                if (ni.Length > 1e-10) ni.Unitize();

                double minR = maxClearance;
                for (int j = 0; j < nV; j += repStride)
                {
                    if (oneRing[i].Contains(j)) continue;
                    Vector3d diff = activePos[j] - pi;
                    double d2 = diff.SquareLength;
                    if (d2 < 1e-12) continue;

                    // Tangent-Point sphere radius R_TP = ||x_i - x_j||^2 / (2 |n_i . (x_i - x_j)|)
                    double normProj = Math.Abs(ni * diff);
                    double rTp = normProj > 1e-6 ? (d2 / (2.0 * normProj)) : Math.Sqrt(d2);
                    double dEuclid = Math.Sqrt(d2);
                    double rEff = Math.Min(rTp, dEuclid);
                    if (rEff < minR) minR = rEff;
                }

                selfClearanceRadii[i] = minR;
                double contactStress = minR < shellThickness * 1.4
                    ? Math.Min(1.0, (shellThickness * 1.4 - minR) / Math.Max(1e-5, shellThickness * 1.4))
                    : 0.0;
                stressField[i] = contactStress;

                if (contactStress > 0.35)
                    nearSelfContactPoints.Add(pi);
            }

            // Color activeShell by self-collision stress (Cool Teal = high clearance, Gold/Crimson = near self-contact)
            activeShell.VertexColors.Clear();
            for (int i = 0; i < nV; i++)
            {
                activeShell.VertexColors.Add(EvaluateScientificColor(stressField[i], 0));
            }

            // 6. Extract sample vertex folding trajectories from State A to State B
            int trajStride = Math.Max(1, nV / 45);
            for (int i = 0; i < nV; i += trajStride)
            {
                var pts = new List<Point3d>(nFrames);
                for (int fIdx = 0; fIdx < nFrames; fIdx++)
                    pts.Add(allFramePositions[fIdx][i]);
                vertexTrajectories.Add(new Polyline(pts));
            }

            return activeShell;
        }

        // =========================================================================================
        // 32. FAST POINTWISE SIGNED DISTANCE FOR POINT CLOUDS & POINT CLOUD DENOISING / CLEANING
        //     Inspired by "Fast Pointwise Signed Distance for Point Clouds" / "Points as Tori"
        //     & "A Heat Method for Generalized Signed Distance" (Feng & Crane 2024).
        //     - Rejects statistical outliers from raw point clouds
        //     - Estimates PCA tangent planes & globally consistent dipole normals
        //     - Evaluates Fast Pointwise Signed Distance d_+-(x) and projects noisy points onto d=0
        //     - Optionally reconstructs a clean watertight 3D Mesh from the zero-isosurface!
        // =========================================================================================
        public static List<Point3d> ComputeFastPointCloudCleanAndSignedDistance(
            IList<Point3d> rawPoints,
            IList<Point3d> optionalQueryPoints,
            double outlierAndNoiseFilter,
            int reconstructGridRes,
            out List<Vector3d> orientedNormals,
            out List<Point3d> rejectedOutliers,
            out double[] evaluatedSignedDistances,
            out Vector3d[] evaluatedSdfGradients,
            out Mesh reconstructedSurfaceMesh)
        {
            orientedNormals = new List<Vector3d>();
            rejectedOutliers = new List<Point3d>();
            evaluatedSignedDistances = Array.Empty<double>();
            evaluatedSdfGradients = Array.Empty<Vector3d>();
            reconstructedSurfaceMesh = new Mesh();
            var cleanedPoints = new List<Point3d>();

            if (rawPoints == null || rawPoints.Count == 0)
                return cleanedPoints;

            int nRaw = rawPoints.Count;

            // 1. Subsample reference set if cloud is huge (> 3000 pts) to keep k-NN & dipole evaluation interactive
            int refStride = Math.Max(1, nRaw / 1500);
            var refPts = new List<Point3d>();
            for (int i = 0; i < nRaw; i += refStride)
                refPts.Add(rawPoints[i]);
            int nRef = refPts.Count;

            // Compute k-NN mean distance per point for statistical outlier rejection
            int kNN = Math.Min(10, Math.Max(3, nRef - 1));
            double[] meanKnnDist = new double[nRaw];
            double sumDist = 0.0;

            for (int i = 0; i < nRaw; i++)
            {
                Point3d pi = rawPoints[i];
                // Maintain top-k smallest squared distances to refPts
                double[] bestD2 = new double[kNN];
                for (int k = 0; k < kNN; k++) bestD2[k] = double.MaxValue;

                for (int j = 0; j < nRef; j++)
                {
                    double d2 = pi.DistanceToSquared(refPts[j]);
                    if (d2 < 1e-14) continue; // skip self
                    if (d2 < bestD2[kNN - 1])
                    {
                        int ins = kNN - 1;
                        while (ins > 0 && d2 < bestD2[ins - 1])
                        {
                            bestD2[ins] = bestD2[ins - 1];
                            ins--;
                        }
                        bestD2[ins] = d2;
                    }
                }

                double avgD = 0.0;
                int validK = 0;
                for (int k = 0; k < kNN; k++)
                {
                    if (bestD2[k] < double.MaxValue * 0.5)
                    {
                        avgD += Math.Sqrt(bestD2[k]);
                        validK++;
                    }
                }
                meanKnnDist[i] = validK > 0 ? avgD / validK : 0.0;
                sumDist += meanKnnDist[i];
            }

            double globalMeanSpacing = sumDist / Math.Max(1, nRaw);
            double sumVar = 0.0;
            for (int i = 0; i < nRaw; i++)
            {
                double diff = meanKnnDist[i] - globalMeanSpacing;
                sumVar += diff * diff;
            }
            double stdSpacing = Math.Sqrt(sumVar / Math.Max(1, nRaw));

            // Outlier threshold: smaller Tol = stricter outlier rejection; Tol <= 0 keeps all points
            double sigmaMultiplier = outlierAndNoiseFilter > 1e-4
                ? Math.Max(0.8, 3.2 / Math.Sqrt(outlierAndNoiseFilter))
                : 100.0;
            double maxAllowedKnnDist = globalMeanSpacing + sigmaMultiplier * stdSpacing;

            var inlierPts = new List<Point3d>(nRaw);
            Point3d cloudCentroid = Point3d.Origin;
            for (int i = 0; i < nRaw; i++)
            {
                if (outlierAndNoiseFilter > 1e-4 && meanKnnDist[i] > maxAllowedKnnDist)
                {
                    rejectedOutliers.Add(rawPoints[i]);
                }
                else
                {
                    inlierPts.Add(rawPoints[i]);
                    cloudCentroid += rawPoints[i];
                }
            }

            if (inlierPts.Count == 0)
            {
                inlierPts.AddRange(rawPoints);
                rejectedOutliers.Clear();
            }
            cloudCentroid /= inlierPts.Count;

            int nIn = inlierPts.Count;
            int inRefStride = Math.Max(1, nIn / 1200);
            var dipolePts = new List<Point3d>();
            for (int i = 0; i < nIn; i += inRefStride)
                dipolePts.Add(inlierPts[i]);
            int nDip = dipolePts.Count;

            // 2. Estimate Local PCA Tangent Plane & Consistent Outward Normal at each Inlier & Dipole Point
            double hKernel = Math.Max(1e-5, globalMeanSpacing * 2.2);
            double h2 = hKernel * hKernel;

            Vector3d EstimateNormalAt(Point3d p)
            {
                // Weighted local covariance matrix around p
                Point3d bary = Point3d.Origin;
                double wSum = 0.0;
                for (int j = 0; j < nDip; j++)
                {
                    double d2 = p.DistanceToSquared(dipolePts[j]);
                    if (d2 > 9.0 * h2) continue;
                    double w = 1.0 / (d2 + 0.25 * h2);
                    bary += w * dipolePts[j];
                    wSum += w;
                }
                if (wSum < 1e-12)
                {
                    Vector3d rad = p - cloudCentroid;
                    if (rad.Length > 1e-10) rad.Unitize();
                    else rad = Vector3d.ZAxis;
                    return rad;
                }
                bary /= wSum;

                double cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
                for (int j = 0; j < nDip; j++)
                {
                    double d2 = p.DistanceToSquared(dipolePts[j]);
                    if (d2 > 9.0 * h2) continue;
                    double w = 1.0 / (d2 + 0.25 * h2);
                    Vector3d d = dipolePts[j] - bary;
                    cxx += w * d.X * d.X;
                    cxy += w * d.X * d.Y;
                    cxz += w * d.X * d.Z;
                    cyy += w * d.Y * d.Y;
                    cyz += w * d.Y * d.Z;
                    czz += w * d.Z * d.Z;
                }

                // Inverse power / cross-product extraction of smallest eigenvector (normal to local tangent plane)
                Vector3d row0 = new Vector3d(cxx, cxy, cxz);
                Vector3d row1 = new Vector3d(cxy, cyy, cyz);
                Vector3d row2 = new Vector3d(cxz, cyz, czz);

                Vector3d c01 = Vector3d.CrossProduct(row0, row1);
                Vector3d c02 = Vector3d.CrossProduct(row0, row2);
                Vector3d c12 = Vector3d.CrossProduct(row1, row2);

                Vector3d bestN = c01;
                if (c02.SquareLength > bestN.SquareLength) bestN = c02;
                if (c12.SquareLength > bestN.SquareLength) bestN = c12;

                if (bestN.Length < 1e-12) bestN = p - cloudCentroid;
                if (bestN.Length < 1e-12) bestN = Vector3d.ZAxis;
                bestN.Unitize();

                // Orient consistently outward relative to cloud centroid + local curvature normal
                Vector3d outward = p - cloudCentroid;
                if ((bestN * outward) < 0.0) bestN = -bestN;
                return bestN;
            }

            var dipoleNormals = new Vector3d[nDip];
            for (int j = 0; j < nDip; j++)
                dipoleNormals[j] = EstimateNormalAt(dipolePts[j]);

            // Smooth dipole normals across neighbors for globally coherent Winding / Signed Distance sign!
            for (int pass = 0; pass < 3; pass++)
            {
                var nextN = new Vector3d[nDip];
                for (int i = 0; i < nDip; i++)
                {
                    Vector3d acc = dipoleNormals[i] * 2.0;
                    for (int j = 0; j < nDip; j++)
                    {
                        double d2 = dipolePts[i].DistanceToSquared(dipolePts[j]);
                        if (d2 > 6.0 * h2 || i == j) continue;
                        double w = 1.0 / (d2 + 0.2 * h2);
                        if ((dipoleNormals[i] * dipoleNormals[j]) >= 0.0)
                            acc += w * dipoleNormals[j];
                        else
                            acc -= w * dipoleNormals[j];
                    }
                    if (acc.Length > 1e-12) acc.Unitize();
                    else acc = dipoleNormals[i];

                    if ((acc * (dipolePts[i] - cloudCentroid)) < -0.15)
                        acc = -acc;
                    nextN[i] = acc;
                }
                dipoleNormals = nextN;
            }

            // 3. Fast Pointwise Signed Distance Evaluator d_+- (Feng & Crane "Fast Pointwise Signed Distance for Point Clouds")
            //    Evaluates singular dipole / algebraic sphere kernel:
            //    d_+- (q) = [ sum_j w_j(q) * n_j . (q - p_j) ] / [ sum_j w_j(q) ]
            double EvaluateFastPointwiseSDF(Point3d q, out Vector3d gradSdf)
            {
                double num = 0.0;
                double den = 0.0;
                Vector3d nAvg = Vector3d.Zero;
                double minD2 = double.MaxValue;
                int closestJ = 0;

                double eps2 = 0.12 * h2;
                for (int j = 0; j < nDip; j++)
                {
                    Vector3d diff = q - dipolePts[j];
                    double d2 = diff.SquareLength;
                    if (d2 < minD2)
                    {
                        minD2 = d2;
                        closestJ = j;
                    }

                    double inv = 1.0 / (d2 + eps2);
                    double w = inv * inv; // 1 / (||q - p_j||^2 + eps^2)^2 singular dipole weight
                    num += w * (dipoleNormals[j] * diff);
                    den += w;
                    nAvg += w * dipoleNormals[j];
                }

                if (den < 1e-14)
                {
                    gradSdf = dipoleNormals[closestJ];
                    return (q - dipolePts[closestJ]) * gradSdf;
                }

                gradSdf = nAvg / den;
                if (gradSdf.Length > 1e-12) gradSdf.Unitize();
                else gradSdf = dipoleNormals[closestJ];

                double signedProj = num / den;
                // Blend near-field exact plane projection with far-field Euclidean distance * sign
                double euclidDist = Math.Sqrt(minD2);
                double blendFar = Math.Min(1.0, euclidDist / (3.0 * hKernel));
                double sign = signedProj >= 0.0 ? 1.0 : -1.0;
                return (1.0 - blendFar) * signedProj + blendFar * (sign * euclidDist);
            }

            // 4. Project Inlier Points onto the Smooth Zero-Isosurface d_+- = 0 (Point Cloud Denoising!)
            double projAlpha = Math.Min(1.0, Math.Max(0.0, outlierAndNoiseFilter * 0.42));
            var rawDisplacements = new double[nIn];
            var inlierGrads = new Vector3d[nIn];

            for (int i = 0; i < nIn; i++)
            {
                Point3d p = inlierPts[i];
                double sdf = EvaluateFastPointwiseSDF(p, out Vector3d gSdf);
                Point3d pClean = p - (projAlpha * sdf) * gSdf;

                cleanedPoints.Add(pClean);
                orientedNormals.Add(gSdf);
                rawDisplacements[i] = sdf;
                inlierGrads[i] = gSdf;
            }

            // 5. Evaluate Pointwise Signed Distance at Query Points Q (or return cloud SDF values if Q is empty)
            if (optionalQueryPoints != null && optionalQueryPoints.Count > 0)
            {
                int nQ = optionalQueryPoints.Count;
                evaluatedSignedDistances = new double[nQ];
                evaluatedSdfGradients = new Vector3d[nQ];
                for (int q = 0; q < nQ; q++)
                {
                    evaluatedSignedDistances[q] = EvaluateFastPointwiseSDF(optionalQueryPoints[q], out Vector3d g);
                    evaluatedSdfGradients[q] = g;
                }
            }
            else
            {
                evaluatedSignedDistances = rawDisplacements;
                evaluatedSdfGradients = inlierGrads;
            }

            // 6. Optional Watertight Surface Mesh Reconstruction from Fast Pointwise Signed Distance (d_+- = 0)
            if (reconstructGridRes >= 6 && cleanedPoints.Count >= 4)
            {
                int res = Math.Min(30, reconstructGridRes);
                BoundingBox box = new BoundingBox(cleanedPoints);
                Vector3d pad = new Vector3d(hKernel * 1.8, hKernel * 1.8, hKernel * 1.8);
                Point3d bMin = box.Min - pad;
                Point3d bMax = box.Max + pad;
                Vector3d bSpan = bMax - bMin;

                double dx = Math.Max(1e-4, bSpan.X / res);
                double dy = Math.Max(1e-4, bSpan.Y / res);
                double dz = Math.Max(1e-4, bSpan.Z / res);

                int nx = res + 1, ny = res + 1, nz = res + 1;
                Point3d[,,] gPts = new Point3d[nx, ny, nz];
                double[,,] gVal = new double[nx, ny, nz];

                for (int ix = 0; ix < nx; ix++)
                {
                    for (int iy = 0; iy < ny; iy++)
                    {
                        for (int iz = 0; iz < nz; iz++)
                        {
                            Point3d gp = new Point3d(bMin.X + ix * dx, bMin.Y + iy * dy, bMin.Z + iz * dz);
                            gPts[ix, iy, iz] = gp;
                            gVal[ix, iy, iz] = EvaluateFastPointwiseSDF(gp, out _);
                        }
                    }
                }

                // Extract d_+- = 0 zero-level set via Kuhn 6-Tetrahedra Marching Tetrahedra
                int[,] kuhnTets = new int[,]
                {
                    { 0, 5, 1, 6 }, { 0, 1, 2, 6 }, { 0, 2, 3, 6 },
                    { 0, 3, 7, 6 }, { 0, 7, 4, 6 }, { 0, 4, 5, 6 }
                };

                Point3d[] cP = new Point3d[8];
                double[] cV = new double[8];
                Point3d[] tP = new Point3d[4];
                double[] tV = new double[4];

                for (int ix = 0; ix < res; ix++)
                {
                    for (int iy = 0; iy < res; iy++)
                    {
                        for (int iz = 0; iz < res; iz++)
                        {
                            cP[0] = gPts[ix, iy, iz];         cV[0] = gVal[ix, iy, iz];
                            cP[1] = gPts[ix + 1, iy, iz];     cV[1] = gVal[ix + 1, iy, iz];
                            cP[2] = gPts[ix + 1, iy + 1, iz]; cV[2] = gVal[ix + 1, iy + 1, iz];
                            cP[3] = gPts[ix, iy + 1, iz];     cV[3] = gVal[ix, iy + 1, iz];
                            cP[4] = gPts[ix, iy, iz + 1];     cV[4] = gVal[ix, iy, iz + 1];
                            cP[5] = gPts[ix + 1, iy, iz + 1]; cV[5] = gVal[ix + 1, iy, iz + 1];
                            cP[6] = gPts[ix + 1, iy + 1, iz + 1]; cV[6] = gVal[ix + 1, iy + 1, iz + 1];
                            cP[7] = gPts[ix, iy + 1, iz + 1]; cV[7] = gVal[ix, iy + 1, iz + 1];

                            bool allPos = true, allNeg = true;
                            for (int c = 0; c < 8; c++)
                            {
                                if (cV[c] <= 0.0) allPos = false;
                                if (cV[c] >= 0.0) allNeg = false;
                            }
                            if (allPos || allNeg) continue;

                            for (int t = 0; t < 6; t++)
                            {
                                for (int tv = 0; tv < 4; tv++)
                                {
                                    int cid = kuhnTets[t, tv];
                                    tP[tv] = cP[cid];
                                    tV[tv] = cV[cid];
                                }
                                AddMarchingTetIsoTriangles(
                                    reconstructedSurfaceMesh,
                                    tP[0], tP[1], tP[2], tP[3],
                                    tV[0], tV[1], tV[2], tV[3]);
                            }
                        }
                    }
                }

                reconstructedSurfaceMesh.Weld(Math.PI);
                reconstructedSurfaceMesh.Normals.ComputeNormals();
            }

            return cleanedPoints;
        }
    }
}
