using System;
using System.Collections.Generic;

namespace CraneGH.Core
{
    /// <summary>
    /// High-performance self-contained Sparse Matrix (CSR) and Linear/Eigen Solver
    /// tailored for Discrete Differential Geometry (Cotan Laplacians, Heat Diffusion,
    /// Connection Laplacians, and Inverse Power Method for Stripe/Direction Eigenproblems).
    /// Zero external native DLL dependencies.
    /// </summary>
    public class CsrMatrix
    {
        public readonly int Rows;
        public readonly int Cols;
        public readonly int[] RowPtr;
        public readonly int[] ColIdx;
        public readonly double[] Values;

        public CsrMatrix(int rows, int cols, int[] rowPtr, int[] colIdx, double[] values)
        {
            Rows = rows;
            Cols = cols;
            RowPtr = rowPtr;
            ColIdx = colIdx;
            Values = values;
        }

        public void Multiply(double[] x, double[] y)
        {
            for (int i = 0; i < Rows; i++)
            {
                double sum = 0.0;
                int start = RowPtr[i];
                int end = RowPtr[i + 1];
                for (int k = start; k < end; k++)
                {
                    sum += Values[k] * x[ColIdx[k]];
                }
                y[i] = sum;
            }
        }

        public double[] ExtractDiagonal()
        {
            int n = Math.Min(Rows, Cols);
            double[] diag = new double[n];
            for (int i = 0; i < n; i++)
            {
                diag[i] = 1.0;
                int start = RowPtr[i];
                int end = RowPtr[i + 1];
                for (int k = start; k < end; k++)
                {
                    if (ColIdx[k] == i)
                    {
                        diag[i] = Values[k];
                        break;
                    }
                }
            }
            return diag;
        }
    }

    public class SparseMatrixBuilder
    {
        public readonly int Rows;
        public readonly int Cols;
        private readonly Dictionary<int, double>[] _rows;

        public SparseMatrixBuilder(int rows, int cols)
        {
            Rows = rows;
            Cols = cols;
            _rows = new Dictionary<int, double>[rows];
            for (int i = 0; i < rows; i++)
                _rows[i] = new Dictionary<int, double>(8);
        }

        public void Add(int r, int c, double val)
        {
            if (Math.Abs(val) < 1e-18) return;
            var row = _rows[r];
            if (row.TryGetValue(c, out double existing))
                row[c] = existing + val;
            else
                row[c] = val;
        }

        public void Set(int r, int c, double val)
        {
            _rows[r][c] = val;
        }

        public void ShiftDiagonal(double shift)
        {
            int n = Math.Min(Rows, Cols);
            for (int i = 0; i < n; i++)
                Add(i, i, shift);
        }

        /// <summary>
        /// Pins a degree of freedom (Dirichlet boundary condition: x[idx] = rhsVal)
        /// by zeroing row idx and setting diagonal to 1.
        /// </summary>
        public void PinRow(int idx, double diagWeight = 1.0)
        {
            _rows[idx].Clear();
            _rows[idx][idx] = diagWeight;
        }

        public CsrMatrix Build()
        {
            int nnz = 0;
            for (int i = 0; i < Rows; i++)
                nnz += _rows[i].Count;

            int[] rowPtr = new int[Rows + 1];
            int[] colIdx = new int[nnz];
            double[] values = new double[nnz];

            int pos = 0;
            var tempCols = new List<int>(16);
            for (int i = 0; i < Rows; i++)
            {
                rowPtr[i] = pos;
                tempCols.Clear();
                foreach (var kv in _rows[i])
                    tempCols.Add(kv.Key);
                tempCols.Sort();

                for (int j = 0; j < tempCols.Count; j++)
                {
                    int col = tempCols[j];
                    colIdx[pos] = col;
                    values[pos] = _rows[i][col];
                    pos++;
                }
            }
            rowPtr[Rows] = pos;
            return new CsrMatrix(Rows, Cols, rowPtr, colIdx, values);
        }
    }

    public static class SparseLinearSolver
    {
        /// <summary>
        /// Solves symmetric positive-definite sparse system A x = b using Jacobi-Preconditioned Conjugate Gradient (PCG).
        /// Extremely fast and robust on Cotan-Laplacian, Heat Diffusion, and Connection Laplacian systems.
        /// </summary>
        public static double[] SolvePCG(CsrMatrix A, double[] b, int maxIter = 1500, double tol = 1e-8, double[] initialGuess = null)
        {
            int n = A.Rows;
            double[] x = new double[n];
            if (initialGuess != null && initialGuess.Length == n)
                Array.Copy(initialGuess, x, n);

            double[] r = new double[n];
            double[] z = new double[n];
            double[] p = new double[n];
            double[] Ap = new double[n];

            // Preconditioner M^-1 = 1 / diag(A)
            double[] invDiag = A.ExtractDiagonal();
            for (int i = 0; i < n; i++)
                invDiag[i] = Math.Abs(invDiag[i]) > 1e-14 ? 1.0 / invDiag[i] : 1.0;

            A.Multiply(x, Ap);
            double normB = 0.0;
            for (int i = 0; i < n; i++)
            {
                r[i] = b[i] - Ap[i];
                z[i] = invDiag[i] * r[i];
                p[i] = z[i];
                normB += b[i] * b[i];
            }
            normB = Math.Sqrt(normB);
            if (normB < 1e-15) return x;

            double rzOld = Dot(r, z);

            for (int iter = 0; iter < maxIter; iter++)
            {
                A.Multiply(p, Ap);
                double pAp = Dot(p, Ap);
                if (Math.Abs(pAp) < 1e-20) break;

                double alpha = rzOld / pAp;
                double resNormSq = 0.0;

                for (int i = 0; i < n; i++)
                {
                    x[i] += alpha * p[i];
                    r[i] -= alpha * Ap[i];
                    resNormSq += r[i] * r[i];
                }

                if (Math.Sqrt(resNormSq) <= tol * Math.Max(1.0, normB))
                    break;

                for (int i = 0; i < n; i++)
                    z[i] = invDiag[i] * r[i];

                double rzNew = Dot(r, z);
                if (Math.Abs(rzOld) < 1e-24) break;

                double beta = rzNew / rzOld;
                for (int i = 0; i < n; i++)
                    p[i] = z[i] + beta * p[i];

                rzOld = rzNew;
            }

            return x;
        }

        /// <summary>
        /// Computes the ground-state eigenvector (smallest eigenvalue) of the generalized eigenvalue problem
        /// A x = lambda M x, where A is symmetric positive-definite (shifted) and M is diagonal mass matrix,
        /// using the Inverse Power Method (Exactly as in Keenan Crane's StripePatterns & GloballyOptimalDirectionFields).
        /// </summary>
        public static double[] SmallestEigenvector(CsrMatrix A, double[] massDiag, int powerIterations = 12, int pcgIter = 600)
        {
            int n = A.Rows;
            var rng = new Random(1234567);
            double[] x = new double[n];
            for (int i = 0; i < n; i++)
                x[i] = rng.NextDouble() * 2.0 - 1.0;

            NormalizeMass(x, massDiag);

            double[] rhs = new double[n];
            for (int step = 0; step < powerIterations; step++)
            {
                for (int i = 0; i < n; i++)
                    rhs[i] = (massDiag != null ? massDiag[i] : 1.0) * x[i];

                x = SolvePCG(A, rhs, pcgIter, 1e-6, x);
                NormalizeMass(x, massDiag);
            }

            return x;
        }

        private static void NormalizeMass(double[] x, double[] massDiag)
        {
            double normSq = 0.0;
            for (int i = 0; i < x.Length; i++)
            {
                double m = massDiag != null ? massDiag[i] : 1.0;
                normSq += x[i] * x[i] * m;
            }
            double invNorm = normSq > 1e-20 ? 1.0 / Math.Sqrt(normSq) : 1.0;
            for (int i = 0; i < x.Length; i++)
                x[i] *= invNorm;
        }

        public static double Dot(double[] a, double[] b)
        {
            double sum = 0.0;
            for (int i = 0; i < a.Length; i++)
                sum += a[i] * b[i];
            return sum;
        }
    }
}
