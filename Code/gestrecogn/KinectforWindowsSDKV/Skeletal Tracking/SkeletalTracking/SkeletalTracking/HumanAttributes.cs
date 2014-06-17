using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace SkeletalTracking
{
    public partial class HumanAttributes
    {
        private float[] rightHand = new float[3];
        private float[] rightWrist = new float[3];
        private float[] rightElbow = new float[3];
        private float[] rightShoulder = new float[3];
        private float[] rightLeg = new float[3];

        private float[] leftHand = new float[3];
        private float[] leftWrist = new float[3];
        private float[] leftElbow = new float[3];
        private float[] leftShoulder = new float[3];
        private float[] leftLeg = new float[3];

        public float[] RightHand{
            get{return rightHand;}
            set{rightHand = value;}
        }

        public float[] RightWrist
        {
            get { return rightWrist; }
            set { rightWrist = value; }
        }
        public float[] RightElbow
        {
            get { return rightElbow; }
            set { rightElbow = value; }
        }
        public float[] RightShoulder
        {
            get { return rightShoulder; }
            set { rightShoulder = value; }
        }

        public float[] LeftHand
        {
            get { return leftHand; }
            set { leftHand = value; }
        }
        public float[] LeftWrist
        {
            get { return leftWrist; }
            set { leftWrist = value; }
        }
        public float[] LeftElbow
        {
            get { return leftElbow; }
            set { leftElbow = value; }
        }
        public float[] LeftShoulder
        {
            get { return leftShoulder; }
            set { leftShoulder = value; }
        }
        public float[] RightLeg
        {
            get { return rightLeg; }
            set { rightLeg= value; }
        }
        public float[] LeftLeg
        {
            get { return leftLeg; }
            set { leftLeg = value; }
        }




    }
}
