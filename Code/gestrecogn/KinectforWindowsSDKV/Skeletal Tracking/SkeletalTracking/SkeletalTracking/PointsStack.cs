using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SkeletalTracking
{
    class PointsStack
    {
        private Stack<coordinates> queue = new Stack<coordinates>();

        public Stack<coordinates> q
        {
            get { return queue; }
            set { queue = value; }
        }
    }
}
