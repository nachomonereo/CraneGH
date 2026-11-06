using System;
using System.Drawing;
using Grasshopper.Kernel;
using CraneGH.Core;

namespace CraneGH
{
    public class CraneGHInfo : GH_AssemblyInfo
    {
        public override string Name => "CraneGH";

        public override Bitmap Icon => CraneIcons.GetIcon("HeatDist");

        public override string Description =>
            "Discrete Differential Geometry (DDG) Mesh Processing Toolkit for Rhino & Grasshopper based on Keenan Crane's Research (CMU).";

        public override Guid Id => new Guid("8F24A9E1-5B17-4D62-9A30-C7E41083B521");

        public override string AuthorName => "CraneGH DDG Toolkit";

        public override string AuthorContact => "https://www.cs.cmu.edu/~kmcrane/";
    }
}
