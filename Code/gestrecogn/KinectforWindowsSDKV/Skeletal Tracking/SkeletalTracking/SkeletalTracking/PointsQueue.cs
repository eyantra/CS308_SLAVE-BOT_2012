using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SkeletalTracking
{
    public struct coordinates{
        public float distance;
        public int angle;
    }
    public partial class PointsQueue
    {
        private Queue<HumanAttributes> queue = new Queue<HumanAttributes>();

        public Queue<HumanAttributes> q
        {
            get { return queue; }
            set { queue = value; }
        }

    }
}
