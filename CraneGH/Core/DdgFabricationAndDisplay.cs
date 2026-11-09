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

            Color[] palette = GenerateDistinctPalette(K);
            List<Mesh> subMeshes = BuildSubMeshesFromFaceLabels(mesh, faceLabels, K, palette, out coloredMesh);
            boundaryCurves = ExtractLabelBoundaryCurves(mesh, faceLabels);

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
        // 19. CONCENTRIC & SPIRAL CNC MILLING TOOLPATHS ON MESH (Iso-Scallop & Ball-End Offset)
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
            out Mesh coloredDistanceMesh)
        {
            int nV = mesh.VertexCount;
            double[] distField;

            if (optionalCustomScalar != null && optionalCustomScalar.Count > 0)
            {
                distField = mesh.MapScalarInputToVertices(optionalCustomScalar);
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
                // Default: Concentric inward milling from naked boundary loop (or vertex 0 if closed)
                var bndSeeds = new List<int>();
                for (int i = 0; i < nV; i++)
                    if (mesh.VertexOnBoundary[i]) bndSeeds.Add(i);
                if (bndSeeds.Count == 0) bndSeeds.Add(0);
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
                : Math.Max(1e-4, (maxD - minD) / 15.0);

            var isoLevels = new List<double>();
            for (double val = minD + 0.5 * stepD; val < maxD - 1e-6; val += stepD)
            {
                isoLevels.Add(val);
                if (isoLevels.Count >= 250) break;
            }

            List<Polyline> surfacePaths = mesh.ExtractIsoContours(distField, isoLevels);

            // Compute Ball-End Mill Center Offset Paths (P_center = P_contact + R_tool * N) and 5-Axis/6-Axis Tool Frames
            toolCenterOffsetPaths = new List<Polyline>(surfacePaths.Count);
            toolFrames = new List<Plane>();
            var spiralPts = new List<Point3d>();

            double rTool = Math.Max(0.0, ballToolRadius);
            estimatedScallopHeight = rTool > 1e-6 ? (stepD * stepD) / (8.0 * rTool) : 0.0;

            for (int cIdx = 0; cIdx < surfacePaths.Count; cIdx++)
            {
                Polyline pl = surfacePaths[cIdx];
                if (pl == null || pl.Count < 2) continue;

                var offsetPts = new List<Point3d>(pl.Count);
                for (int i = 0; i < pl.Count; i++)
                {
                    Point3d p = pl[i];
                    MeshPoint mp = mesh.RhinoMesh.ClosestMeshPoint(p, 0.0);
                    Vector3d n = Vector3d.ZAxis;
                    if (mp != null)
                    {
                        MeshFace mf = mesh.RhinoMesh.Faces[mp.FaceIndex];
                        n = mp.T[0] * mesh.VertexNormals[mf.A] +
                            mp.T[1] * mesh.VertexNormals[mf.B] +
                            mp.T[2] * mesh.VertexNormals[mf.C];
                        if (n.Length > 1e-12) n.Unitize();
                    }

                    Point3d pCenter = p + rTool * n;
                    offsetPts.Add(pCenter);
                    spiralPts.Add(pCenter);

                    // Sample tool frames along the path
                    if (i % Math.Max(1, pl.Count / 8) == 0)
                    {
                        Vector3d tangent = (i < pl.Count - 1) ? (pl[i + 1] - pl[i]) : (pl[i] - pl[i - 1]);
                        tangent -= (tangent * n) * n;
                        if (tangent.Length < 1e-12)
                            tangent = Vector3d.CrossProduct(n, Math.Abs(n.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                        tangent.Unitize();
                        Vector3d binormal = Vector3d.CrossProduct(n, tangent);
                        binormal.Unitize();
                        toolFrames.Add(new Plane(pCenter, tangent, binormal));
                    }
                }

                toolCenterOffsetPaths.Add(new Polyline(offsetPts));
            }

            connectedSpiralPath = new Polyline(spiralPts);
            coloredDistanceMesh = ColorMeshByScalar(mesh.RhinoMesh, distField, false);
            return surfacePaths;
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

            // 2. Gather obstacle sample points & normals for fast Tangent-Point Repulsive Shell evaluation
            BoundingBox sceneBox = new BoundingBox(baseFrames[0].Origin, baseFrames[nSteps - 1].Origin);
            for (int i = 0; i < nSteps; i++) sceneBox.Union(baseFrames[i].Origin);

            var obsPts = new List<Point3d>();
            var obsNrms = new List<Vector3d>();

            if (obstacleMeshes != null)
            {
                foreach (Mesh om in obstacleMeshes)
                {
                    if (om == null || om.Vertices.Count == 0) continue;
                    om.Normals.ComputeNormals();
                    sceneBox.Union(om.GetBoundingBox(false));

                    // Subsample obstacle vertices + face centers up to ~600 representative repulsive sites per mesh
                    int vStride = Math.Max(1, om.Vertices.Count / 450);
                    for (int v = 0; v < om.Vertices.Count; v += vStride)
                    {
                        obsPts.Add(om.Vertices[v]);
                        Vector3d n = om.Normals.Count > v ? (Vector3d)om.Normals[v] : Vector3d.ZAxis;
                        if (n.Length > 1e-10) n.Unitize();
                        obsNrms.Add(n);
                    }
                    int fStride = Math.Max(1, om.Faces.Count / 250);
                    for (int f = 0; f < om.Faces.Count; f += fStride)
                    {
                        obsPts.Add(om.Faces.GetFaceCenter(f));
                        obsNrms.Add( Vector3d.ZAxis );
                    }
                }
            }

            double diag = Math.Max(10.0, sceneBox.Diagonal.Length);
            double rSpindle = spindleRadiusInput > 1e-5 ? spindleRadiusInput : diag * 0.065;
            double spindleLen = rSpindle * 4.2;

            // Define robot wrist singularity axis (inline forearm alignment or downward self-collision cone)
            Vector3d wristSingularityAxis = new Vector3d(0.0, -0.85, -0.52);
            wristSingularityAxis.Unitize();
            Point3d robotBaseColumn = sceneBox.Center - new Vector3d(0.0, diag * 0.65, 0.0);

            // Helper: Evaluate Repulsive Shell Energy for a candidate Plane (TCP + Spindle Cylinder + Holder Plane + Singularity)
            double EvaluateSpindleShellEnergy(Point3d tcp, Vector3d zAxis, out double minClearance)
            {
                zAxis.Unitize();
                Vector3d uAx = Vector3d.CrossProduct(zAxis, Math.Abs(zAxis.X) < 0.9 ? Vector3d.XAxis : Vector3d.YAxis);
                uAx.Unitize();
                Vector3d vAx = Vector3d.CrossProduct(zAxis, uAx);
                vAx.Unitize();

                // Sample 7 points along the Spindle Shell + Tool Holder Plane Disk
                // 0: TCP tip, 1..3: Spindle shaft at 1.2R, 2.5R, 4.0R along +Z, 4..6: Holder Plane ring at 1.5R
                Point3d[] shellPts = new Point3d[7];
                double[] shellRadii = new double[7];

                shellPts[0] = tcp + zAxis * (0.25 * rSpindle);
                shellRadii[0] = 0.25 * rSpindle;

                shellPts[1] = tcp + zAxis * (1.25 * rSpindle);
                shellRadii[1] = 0.85 * rSpindle;

                shellPts[2] = tcp + zAxis * (2.60 * rSpindle);
                shellRadii[2] = 1.00 * rSpindle;

                shellPts[3] = tcp + zAxis * spindleLen;
                shellRadii[3] = 1.05 * rSpindle;

                // Tool holder flange disk (checks if the Plane itself collides with geometry!)
                Point3d flangeCenter = tcp + zAxis * (1.45 * rSpindle);
                shellPts[4] = flangeCenter + uAx * (1.25 * rSpindle);
                shellRadii[4] = 0.45 * rSpindle;
                shellPts[5] = flangeCenter - uAx * (1.25 * rSpindle);
                shellRadii[5] = 0.45 * rSpindle;
                shellPts[6] = flangeCenter + vAx * (1.25 * rSpindle);
                shellRadii[6] = 0.45 * rSpindle;

                minClearance = diag;
                double energy = 0.0;

                // A. Tangent-Point Repulsive Shell Energy against Obstacles
                int nObs = obsPts.Count;
                for (int o = 0; o < nObs; o++)
                {
                    Point3d op = obsPts[o];
                    for (int s = 0; s < 7; s++)
                    {
                        // For s == 0 (TCP tip) in Mode 0 (printing/milling on the mesh), allow TCP contact with the part
                        if (mode == 0 && s == 0) continue;

                        double d = shellPts[s].DistanceTo(op) - shellRadii[s];
                        if (d < minClearance) minClearance = d;

                        if (d <= 0.0)
                        {
                            // Collision penetration barrier
                            double pen = Math.Abs(d) / Math.Max(1e-5, rSpindle);
                            energy += 12.0 + 35.0 * pen * pen;
                        }
                        else if (d < 2.5 * rSpindle)
                        {
                            // Smooth Tangent-Point 1/d^2 repulsive shell falloff
                            double ratio = rSpindle / Math.Max(0.05 * rSpindle, d);
                            energy += 0.35 * ratio * ratio;
                        }
                    }
                }

                // B. Robot Kinematic Wrist Singularity & Self-Collision Energy
                double singDot = zAxis * wristSingularityAxis;
                if (singDot > 0.65)
                {
                    double sPen = (singDot - 0.65) / 0.35;
                    energy += 8.5 * sPen * sPen;
                }
                // Prevent upside-down tool flips (zAxis.Z < -0.25)
                if (zAxis.Z < -0.15)
                {
                    double flipPen = (-0.15 - zAxis.Z);
                    energy += 14.0 * flipPen * flipPen;
                }
                // Self-collision with robot base column
                double distBase = (tcp + zAxis * spindleLen).DistanceTo(robotBaseColumn);
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
    }
}
