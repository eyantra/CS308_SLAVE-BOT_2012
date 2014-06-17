// (c) Copyright Microsoft Corporation.
// This source is subject to the Microsoft Public License (Ms-PL).
// Please see http://go.microsoft.com/fwlink/?LinkID=131993 for details.
// All other rights reserved.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Coding4Fun.Kinect.Wpf;
using SkeletalTracking;
using System.Threading;
using System.Windows.Threading;
using System.Data;
using System.IO.Ports;
using System.Windows.Forms;

namespace SkeletalTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        HumanAttributes playerAttributes = new HumanAttributes();
        float[] finalpoints = new float[3];
        PointsQueue pointsQueue = new PointsQueue();
        PointsStack pointsStack = new PointsStack();
        bool insertPoints = false;
        bool dir_up = true;
        bool dir_down = true;
        bool dir_left = true;
        bool dir_right = true;
        bool directionsignal = false;
        public coordinates coor = new coordinates();
        Thread t;
        Thread p;
        static AutoResetEvent autoEvent = new AutoResetEvent(false);
        static AutoResetEvent ack = new AutoResetEvent(false);
        static AutoResetEvent webcam_ack = new AutoResetEvent(false);
        SerialPort serialPort = new SerialPort();


        /* Webcam on the bot part */
        //private WebCam_Capture.WebCamCapture UserControl1;
        private WebCam_Capture.WebCamCapture WebCamCapture;
        //private System.Windows.Forms.PictureBox pictureBox1;
        private System.Windows.Forms.Label label1;

        //private System.ComponentModel.Container components = null;

        public MainWindow()
        {
            InitializeComponent();
            InitializeComponent_form();
        }

        #region Windows Form Designer generated code
        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent_form()
        {
            this.WebCamCapture = new WebCam_Capture.WebCamCapture();
            //this.pictureBox1 = new System.Windows.Forms.PictureBox();
            this.label1 = new System.Windows.Forms.Label();
            //this.SuspendLayout();
            // 
            // WebCamCapture
            // 
            this.WebCamCapture.CaptureHeight = 240;
            this.WebCamCapture.CaptureWidth = 320;
            // TODO: Code generation for 'this.WebCamCapture.FrameNumber' failed because of Exception 'Invalid Primitive Type: System.UInt64. Only CLS compliant primitive types can be used. Consider using CodeObjectCreateExpression.'.
            this.WebCamCapture.Location = new System.Drawing.Point(17, 17);
            this.WebCamCapture.Name = "WebCamCapture";
            this.WebCamCapture.Size = new System.Drawing.Size(342, 252);
            this.WebCamCapture.TabIndex = 0;
            this.WebCamCapture.TimeToCapture_milliseconds = 100;
            this.WebCamCapture.ImageCaptured += new WebCam_Capture.WebCamCapture.WebCamEventHandler(this.WebCamCapture_ImageCaptured);
            // 
            // pictureBox1
            // 
            /*
            this.pictureBox1.Location = new System.Drawing.Point(800, 800);
            this.pictureBox1.Name = "pictureBox1";
            this.pictureBox1.Size = new System.Drawing.Size(320, 240);
            this.pictureBox1.TabIndex = 0;
            this.pictureBox1.TabStop = false;
             * */
            this.WebCamCapture.TimeToCapture_milliseconds = 20;
        }
        #endregion

        object webcam_source;
        WebCam_Capture.WebcamEventArgs webcam_events;
        private void WebCamCapture_ImageCaptured(object source, WebCam_Capture.WebcamEventArgs e)
        {
            webcam_source = source;
            webcam_events = e;
            // set the picturebox picture
            //this.pictureBox1.Image = e.WebCamImage;
            webcam_ack.Set();
        }


        private void WebCam_viewer()
        {
            while (true)
            {
                webcam_ack.WaitOne();
                System.Drawing.Bitmap dImg = new System.Drawing.Bitmap(webcam_events.WebCamImage);
                System.IO.MemoryStream ms = new System.IO.MemoryStream();
                dImg.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);
                System.Windows.Media.Imaging.BitmapImage bImg = new System.Windows.Media.Imaging.BitmapImage();
                bImg.BeginInit();
                bImg.StreamSource = new System.IO.MemoryStream(ms.ToArray());
                bImg.EndInit();
                contents.Dispatcher.Invoke(                 // dispatcher 
                        System.Windows.Threading.DispatcherPriority.Normal,
                        new Action(
                            delegate()
                            {
                                camera_view.Source = bImg;
                            }
                )
                );
            }
        }


        /* Bot navigation part */
        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
            t = new Thread(EventHandler);
            p = new Thread(WebCam_viewer);
            if (!(t.ThreadState == ThreadState.Running))
            {
                t.Start();
            }
            serialPort = new SerialPort("COM10",11059200);
            serialPort.BaudRate = 9600;
            serialPort.Parity = Parity.None;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.Handshake = Handshake.None;
            serialPort.DtrEnable = false;
            try
            {
                //this.WebCamCapture.Start(0);
            }
            catch (Exception g)
            {
                System.Windows.MessageBox.Show("webcam couldnot be started" + g);
            }



            try
            {
                serialPort.Open();
            }
            catch(Exception f){
                System.Windows.MessageBox.Show(f.ToString());
                Window_Closing(sender, null);
            }

            serialPort.DataReceived += new SerialDataReceivedEventHandler(SignalReceived);
            serialPort.ErrorReceived += new SerialErrorReceivedEventHandler(ErrorReceived);
        }


        /******************************************* Thread function *******************************************************/
   
        private void EventHandler()
        {
            while (true)
            {
                // Computation and sending here.
                while (pointsQueue.q.Count != 0)
                {
                     ComputeCoordinates();
                }

                // backpropagation Algorithm ....... See about this.
                backpropagation();

                // Wait for the signal by the main thread.
                if (pointsQueue.q.Count == 0)
                {
                    autoEvent.WaitOne();
                }
            }
        }


        void backpropagation()
        {
            string sc_d_c = Char.ConvertFromUtf32(0);
            string sc_r_c = Char.ConvertFromUtf32(180);

            coordinates curr_coor = new coordinates();
            curr_coor.distance = 0;
            curr_coor.angle = 180;
            bool flag = false;
            int last_angle = 0;

            contents.Dispatcher.Invoke(                 // dispatcher 
                System.Windows.Threading.DispatcherPriority.Normal,
                new Action(
                    delegate()
                    {
                        cont.Content += "";
                    }
            ));

            while ((pointsStack.q.Count) != 0)
            {
                /***************** TRAVERSE THESE CO-ORDINATES *****************/
                last_angle = 360 - last_angle;
                sc_r_c = Char.ConvertFromUtf32((int)(last_angle/3));
                sc_d_c = Char.ConvertFromUtf32((int)(curr_coor.distance/2));

                /* Angle */
                serialPort.Write(sc_r_c.ToString());
                /* Distance */
                serialPort.Write(sc_d_c.ToString());
                /* Wait for ack */
                ack.WaitOne();

                last_angle = curr_coor.angle;

                curr_coor = pointsStack.q.Pop();

                flag = true;


                contents.Dispatcher.Invoke(                 // dispatcher 
                    System.Windows.Threading.DispatcherPriority.Normal,
                    new Action(
                        delegate()
                        {
                            cont.Content += "Angle : "+last_angle+ "                 Distance : " + curr_coor.distance;
                        }
                ));
            }

            if (flag)
            {
                // Traverse the last return.
                sc_r_c = Char.ConvertFromUtf32((int)((360 - last_angle )/ 3));
                sc_d_c = Char.ConvertFromUtf32((int)(curr_coor.distance / 2));
                /* Angle */
                serialPort.Write(sc_r_c.ToString());
                /* Distance */
                serialPort.Write(sc_d_c.ToString());
                /* Wait for ack */
                ack.WaitOne();

                contents.Dispatcher.Invoke(                 // dispatcher 
                    System.Windows.Threading.DispatcherPriority.Normal,
                    new Action(
                        delegate()
                        {
                            cont.Content += "Angle : " + (360 - last_angle) + "                 Distance : " + curr_coor.distance;
                        }
                ));

                // Finally turn 180 degrees to come to the original position.
                sc_d_c = Char.ConvertFromUtf32(0);
                sc_r_c = Char.ConvertFromUtf32(Math.Abs(180 - curr_coor.angle)/3);

                /* Angle */
                serialPort.Write(sc_r_c.ToString());
                /* Distance */
                serialPort.Write(sc_d_c.ToString());
                /* Wait for ack */
                ack.WaitOne();
            }
        }


        void ComputeCoordinates()
        {

            float[] rightFeet = pointsQueue.q.First<HumanAttributes>().RightLeg;
            float[] leftFeet = pointsQueue.q.First<HumanAttributes>().LeftLeg;
            float[] rightElbow = pointsQueue.q.First<HumanAttributes>().RightElbow;
            float[] leftElbow = pointsQueue.q.First<HumanAttributes>().LeftElbow;
            float[] rightHand = pointsQueue.q.First<HumanAttributes>().RightHand;

            float[] midpoint = new float[3];
            midpoint[0] = (rightFeet[0] + leftFeet[0]) / 2;
            midpoint[1] = (rightFeet[1] + leftFeet[1]) / 2;
            midpoint[2] = (rightFeet[2] + leftFeet[2]) / 2;


            float temp = ((midpoint[1] - rightElbow[1]) / (rightHand[1] - rightElbow[1]));
            finalpoints[0] = temp * (rightHand[0] - rightElbow[0]) + rightElbow[0];
            finalpoints[1] = midpoint[1];
            finalpoints[2] = temp * (rightHand[2] - rightElbow[2]) + rightElbow[2];

            finalpoints[0] = (midpoint[0] - finalpoints[0]);
            finalpoints[1] = (midpoint[1] - finalpoints[1]);
            finalpoints[2] = (midpoint[2] - finalpoints[2]);
            

            
            //int leftrotmultiplier = 22;   // for rotating 90 or pi/2 degrees
            //int rightrotmultiplier = 22;  //

            float slope = finalpoints[2] / finalpoints[0];
            double angle = Math.Atan(slope);
            double cos_angle = Math.Cos(angle);

            int rotation;

            if (finalpoints[0] > 0 && finalpoints[2] > 0)
            {
                /************** The angle is in first quadrant *********/
                rotation = (int)((angle * 180) / Math.PI);
            }
            else if (finalpoints[0] < 0 && finalpoints[2] < 0)
            {
                /************** The angle is in third quadrant *********/
                rotation = (int)((angle * 180) / Math.PI);
                rotation = 180 + rotation;
            }
            else if (finalpoints[0] < 0 && finalpoints[2] > 0)
            {
                /************** The angle is in second quadrant *********/
                rotation = (int)((angle * 180) / Math.PI);
                rotation = 180 + rotation;
            }
            else
            {
                /************** The angle is in fourth quadrant *********/
                rotation = (int)((angle * 180) / Math.PI);
                rotation = 360 + rotation;
            }

            int distance = (int)(Math.Sqrt(Math.Pow(finalpoints[0], 2) + Math.Pow(finalpoints[2], 2)));
            distance = distance / 10;
            rotation = rotation - 90;
            if (rotation < 0)
            {
                rotation = 360 + rotation;
            }

            if (distance < 255)
            {
                double forward = distance;
                contents.Dispatcher.Invoke(                 // dispatcher 
                    System.Windows.Threading.DispatcherPriority.Normal,
                    new Action(
                        delegate()
                        {
                            gotoX.Content = "gotoX = " + finalpoints[0].ToString();
                            gotoD.Content = "gotoZ = " + finalpoints[2].ToString();
                            Coord1.Content += "Rotation-char:" + Char.ConvertFromUtf32((int)rotation) + "              : " + (int)(rotation) ;
                            Coord1.Content += "      Distance:      " + (int)(distance);
                        }
                ));

                

                int scaling_distance = distance / 2;
                int scaling_rotation = rotation / 3;

                string sc_d_c = Char.ConvertFromUtf32(scaling_distance);
                string sc_r_c = Char.ConvertFromUtf32(scaling_rotation);
                //char g = System.Convert.

                // to send forward and angle i.e scaling_distance & sccaling_rotation
                try
                {
                    /* Angle */
                    serialPort.Write(sc_r_c.ToString());

                    /* Distance */
                    serialPort.Write(sc_d_c.ToString());


                    /* Wait for ack */
                    ack.WaitOne();

                    
                    /* Push these co-ordinates onto the back propagation stack. */
                    coordinates coor = new coordinates();
                    coor.distance = distance;
                    coor.angle = rotation;
                    pointsStack.q.Push(coor);

                }
                catch (Exception f)
                {
                    System.Windows.MessageBox.Show(f.ToString());
                }
            }
            else
            {
                System.Windows.MessageBox.Show("Error: Distance out of range");
            }

            /* Delete the co-ordinates from the queue */
            pointsQueue.q.Dequeue();
        }
        /*
        void GetSteeringdirection()
        {
            if (playerAttributes.RightWrist[1] > playerAttributes.LeftElbow[1])
            {
                //move forward direction
            }
            insertPoints = false;
            directionsignal = true;
        }
        */


        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor old = (KinectSensor)e.OldValue;

            StopKinect(old);

            KinectSensor sensor = (KinectSensor)e.NewValue;

            if (sensor == null)
            {
                return;
            }

            


            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            sensor.SkeletonStream.Enable(parameters);

            sensor.SkeletonStream.Enable();

            sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30); 
            sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

            try
            {
                sensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
        }

        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (closing)
            {
                return;
            }

            //Get a skeleton
            Skeleton first =  GetFirstSkeleton(e);

            if (first == null)
            {
                return; 
            }



            //set scaled position
            //ScalePosition(headImage, first.Joints[JointType.Head]);
            ScalePosition(leftHand, first.Joints[JointType.HandLeft]);
            ScalePosition(leftWrist, first.Joints[JointType.WristLeft]);
            ScalePosition(rightHand, first.Joints[JointType.HandRight]);

            GetCameraPoint(first, e); 

        }

        


        void GetCameraPoint(Skeleton first, AllFramesReadyEventArgs e)
        {

            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                if (depth == null ||
                    kinectSensorChooser1.Kinect == null)
                {
                    return;
                }
                

                //Map a joint location to a point on the depth map
                //head
                
                DepthImagePoint headDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.Head].Position);
               
                //left hand
                DepthImagePoint leftDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.HandLeft].Position);
                playerAttributes.LeftHand[0] = leftDepthPoint.X;
                playerAttributes.LeftHand[1] = leftDepthPoint.Y;
                playerAttributes.LeftHand[2] = leftDepthPoint.Depth;

                //wrist left
                DepthImagePoint leftWristDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.WristLeft].Position);
                playerAttributes.LeftWrist[0] = leftWristDepthPoint.X;
                playerAttributes.LeftWrist[1] = leftWristDepthPoint.Y;
                playerAttributes.LeftWrist[2] = leftWristDepthPoint.Depth;
                //elbow left
                DepthImagePoint leftelbowDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.ElbowLeft].Position);
                playerAttributes.LeftElbow[0] = leftelbowDepthPoint.X;
                playerAttributes.LeftElbow[1] = leftelbowDepthPoint.Y;
                playerAttributes.LeftElbow[2] = leftelbowDepthPoint.Depth;
                //shoulder left
                DepthImagePoint leftShoulderDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.ShoulderLeft].Position);
                playerAttributes.LeftShoulder[0] = leftShoulderDepthPoint.X;
                playerAttributes.LeftShoulder[1] = leftShoulderDepthPoint.Y;
                playerAttributes.LeftShoulder[2] = leftShoulderDepthPoint.Depth;

               
                //right hand
                DepthImagePoint rightDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position);
                playerAttributes.RightHand[0] = rightDepthPoint.X;
                playerAttributes.RightHand[1] = rightDepthPoint.Y;
                playerAttributes.RightHand[2] = rightDepthPoint.Depth;
                    

                //right wrist 
                DepthImagePoint rightWristDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.WristRight].Position);
                playerAttributes.RightWrist[0] = rightWristDepthPoint.X;
                playerAttributes.RightWrist[1] = rightWristDepthPoint.Y;
                playerAttributes.RightWrist[2] = rightWristDepthPoint.Depth;
                //right elbow
                DepthImagePoint rightelbowDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.ElbowRight].Position);
                playerAttributes.RightElbow[0] = rightelbowDepthPoint.X;
                playerAttributes.RightElbow[1] = rightelbowDepthPoint.Y;
                playerAttributes.RightElbow[2] = rightelbowDepthPoint.Depth;
                //right shoulder
                DepthImagePoint rightShoulderDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.ShoulderRight].Position);
                playerAttributes.RightShoulder[0] = rightShoulderDepthPoint.X;
                playerAttributes.RightShoulder[1] = rightShoulderDepthPoint.Y;
                playerAttributes.RightShoulder[2] = rightShoulderDepthPoint.Depth;

                //right leg
                DepthImagePoint rightLegDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.FootRight].Position);
                playerAttributes.RightLeg[0] = rightLegDepthPoint.X;
                playerAttributes.RightLeg[1] = rightLegDepthPoint.Y;
                playerAttributes.RightLeg[2] = rightLegDepthPoint.Depth;

                //left leg
                DepthImagePoint leftLegDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.FootLeft].Position);
                playerAttributes.LeftLeg[0] = leftLegDepthPoint.X;
                playerAttributes.LeftLeg[1] = leftLegDepthPoint.Y;
                playerAttributes.LeftLeg[2] = leftLegDepthPoint.Depth;


                //Compute the 2d Co-ordinates Using joint co-ordinates; 
                


                //Map a depth point to a point on the color image
                // right leg
                ColorImagePoint rightFeetColorPoint =
                    depth.MapToColorImagePoint(rightLegDepthPoint.X, rightLegDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //left leg
                ColorImagePoint leftFeetColorPoint =
                    depth.MapToColorImagePoint(leftLegDepthPoint.X, leftLegDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //head
                ColorImagePoint headColorPoint =
                    depth.MapToColorImagePoint(headDepthPoint.X, headDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //left hand
                ColorImagePoint leftColorPoint =
                    depth.MapToColorImagePoint(leftDepthPoint.X, leftDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //left Wrist
                ColorImagePoint leftWristColorPoint =
                    depth.MapToColorImagePoint(leftWristDepthPoint.X, leftWristDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //left Elbow
                ColorImagePoint leftElbowColorPoint =
                    depth.MapToColorImagePoint(leftelbowDepthPoint.X, leftelbowDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //left shoulder
                ColorImagePoint leftShoulderColorPoint =
                    depth.MapToColorImagePoint(leftShoulderDepthPoint.X, leftShoulderDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);

                //right hand
                ColorImagePoint rightColorPoint =
                    depth.MapToColorImagePoint(rightDepthPoint.X, rightDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //right Wrist
                ColorImagePoint rightWristColorPoint =
                    depth.MapToColorImagePoint(rightWristDepthPoint.X, rightWristDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //right Elbow
                ColorImagePoint rightElbowColorPoint =
                    depth.MapToColorImagePoint(rightelbowDepthPoint.X, rightelbowDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //right Shoulder
                ColorImagePoint rightShoulderColorPoint =
                    depth.MapToColorImagePoint(rightShoulderDepthPoint.X, rightShoulderDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);

                gesturelock.Content = "Working for now";

                string code = "";

                if (playerAttributes.LeftHand[1] < playerAttributes.LeftShoulder[1] && playerAttributes.RightHand[1] > playerAttributes.RightElbow[1])
                {
                    gesturelock.Content = "";
                    gesturelock.Content = playerAttributes.RightWrist[1].ToString() + playerAttributes.RightShoulder[1].ToString();
                    //Action to be taken
                    if (playerAttributes.RightWrist[1] > playerAttributes.RightShoulder[1])
                    {
                        //ComputeCoordinates();
                        insertPoints = true;
                        gesturelock.Content += "Gesture is Locked";
                    }
                    else
                    {
                        //GetSteeringdirection();
                        gesturelock.Content += "Gesture is getdir Locked";
                    }

                    dir_up = true;
                    dir_down = true;
                    dir_left = true;
                    dir_right = true;
                }
                else if (playerAttributes.RightHand[1] < playerAttributes.RightElbow[1] && playerAttributes.LeftHand[1] > playerAttributes.LeftElbow[1])
                {
                    // Now traverse the co-ordinates.
                    autoEvent.Set();

                    dir_up = true;
                    dir_down = true;
                    dir_left = true;
                    dir_right = true;
                }
                else if (playerAttributes.RightHand[2] < playerAttributes.RightShoulder[2] && playerAttributes.LeftHand[2] < playerAttributes.LeftShoulder[2] && playerAttributes.LeftHand[1] < playerAttributes.LeftShoulder[1] && playerAttributes.RightHand[1] < playerAttributes.RightShoulder[1])
                {
                    if (dir_up)
                    {
                        // Just keep sending the forward motion both the hands above the elbow and elbow below the shoulder and beyond the shoulder
                        code = Char.ConvertFromUtf32(90);
                        try
                        {

                            serialPort.Write(code.ToString());
                            serialPort.Write(code.ToString());

                            ack.WaitOne();

                        }
                        catch (Exception f)
                        {
                            System.Windows.MessageBox.Show(f.ToString());
                        }

                        dir_up = false;
                        dir_down = true;
                        dir_left = true;
                        dir_right = true;
                    }
                }
                else if (playerAttributes.RightHand[2] > playerAttributes.RightShoulder[2] && playerAttributes.LeftHand[2] > playerAttributes.LeftShoulder[2] && playerAttributes.LeftHand[1] < playerAttributes.LeftShoulder[1] && playerAttributes.RightHand[1] < playerAttributes.RightShoulder[1])
                {
                    if (dir_down)
                    {
                        // To stop the motion both the hands above the elbow and elbow below the shoulder and within the shoulder
                        code = Char.ConvertFromUtf32(89);
                        try
                        {
                            serialPort.Write(code.ToString());
                            serialPort.Write(code.ToString());

                            ack.WaitOne();
                        }
                        catch (Exception f)
                        {
                            System.Windows.MessageBox.Show(f.ToString());
                        }

                        dir_up = true;
                        dir_down = false;
                        dir_left = true;
                        dir_right = true;
                    }
                }
                else if (playerAttributes.RightHand[2] > playerAttributes.RightShoulder[2] && playerAttributes.LeftHand[2] < playerAttributes.LeftShoulder[2] && playerAttributes.LeftHand[1] < playerAttributes.LeftShoulder[1] && playerAttributes.RightHand[1] < playerAttributes.RightShoulder[1])
                {
                    if (dir_left)
                    {
                        // To turn left same as above but left elbow out of shoulder
                        code = Char.ConvertFromUtf32(88);
                        try
                        {
                            serialPort.Write(code.ToString());
                            serialPort.Write(code.ToString());
                            ack.WaitOne();
                        }
                        catch (Exception f)
                        {
                            System.Windows.MessageBox.Show(f.ToString());
                        }

                        dir_up = true;
                        dir_down = true;
                        dir_left = false;
                        dir_right = true;
                    }
                }
                else if (playerAttributes.RightHand[2] < playerAttributes.RightShoulder[2] && playerAttributes.LeftHand[2] > playerAttributes.LeftShoulder[2] && playerAttributes.LeftHand[1] < playerAttributes.LeftShoulder[1] && playerAttributes.RightHand[1] < playerAttributes.RightShoulder[1])
                {
                    if (dir_right)
                    {
                        // To turn right same as above right elbow out of shoulder
                        code = Char.ConvertFromUtf32(87);
                        try
                        {
                            serialPort.Write(code.ToString());
                            serialPort.Write(code.ToString());
                            ack.WaitOne();
                        }
                        catch (Exception f)
                        {
                            System.Windows.MessageBox.Show(f.ToString());
                        }

                        dir_up = true;
                        dir_down = true;
                        dir_left = true;
                        dir_right = false;
                    }
                }
                    /*
                else if (playerAttributes.LeftHand[1] < playerAttributes.LeftElbow[1] && playerAttributes.RightHand[1] > playerAttributes.RightElbow[1])
                {
                    try
                    {
                        serialPort.Write("5");
                    }
                    catch (Exception f)
                    {
                        System.Windows.MessageBox.Show(f.ToString());
                    }
                }
                  * */
                else
                {

                    /*if (dir_down)
                    {
                        // To stop the motion both the hands above the elbow and elbow below the shoulder and within the shoulder
                        code = Char.ConvertFromUtf32(89);
                        try
                        {
                            serialPort.Write(code.ToString());
                            serialPort.Write(code.ToString());

                            ack.WaitOne();
                        }
                        catch (Exception f)
                        {
                            System.Windows.MessageBox.Show(f.ToString());
                        }

                        dir_up = true;
                        dir_down = false;
                        dir_left = true;
                        dir_right = true;
                    }
                    */
                    

                    if (insertPoints)
                    {
                        // inserting points into queue
                        pointsQueue.q.Enqueue(playerAttributes);
                        playerAttributes = new HumanAttributes();
                        insertPoints = false;
                        dir_up = true;
                        dir_down = true;
                        dir_left = true;
                        dir_right = true;
                    }
                    gesturelock.Content = "Gesture is Not Locked";
                }

                //System.Windows.MessageBox.Show("gothere");

                //Set location
                CameraPosition(Headimage, headColorPoint);
                CameraPosition(leftHand, leftColorPoint);
                CameraPosition(rightHand, rightColorPoint);
                CameraPosition(leftWrist, leftWristColorPoint);
                CameraPosition(leftElbow, leftElbowColorPoint);
                CameraPosition(leftShoulder, leftShoulderColorPoint);
                CameraPosition(rightWrist, rightWristColorPoint);
                CameraPosition(rightElbow, rightElbowColorPoint);
                CameraPosition(rightShoulder, rightShoulderColorPoint);
                CameraPosition(rightFeet, rightFeetColorPoint);
                CameraPosition(leftFeet, leftFeetColorPoint);

            }        
        }


        Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null; 
                }

                
                skeletonFrameData.CopySkeletonDataTo(allSkeletons);

                //get the first tracked skeleton
                Skeleton first = (from s in allSkeletons
                                         where s.TrackingState == SkeletonTrackingState.Tracked
                                         select s).FirstOrDefault();

                return first;

            }
        }

        private void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }


                }
            }
        }

        private void CameraPosition(FrameworkElement element, ColorImagePoint point)
        {
            //Divide by 2 for width and height so point is right in the middle 
            // instead of in top/left corner

            Canvas.SetLeft(element, point.X - element.Width / 2);
            Canvas.SetTop(element, point.Y - element.Height / 2);

        }

        private void ScalePosition(FrameworkElement element, Joint joint)
        {
            //convert the value to X/Y
            //Joint scaledJoint = joint.ScaleTo(1280, 720); 
            
            //convert & scale (.3 = means 1/3 of joint distance)
            Joint scaledJoint = joint.ScaleTo(1280, 720, .3f, .3f);

            Canvas.SetLeft(element, scaledJoint.Position.X);
            Canvas.SetTop(element, scaledJoint.Position.Y); 
            
        }


        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            closing = true; 
            StopKinect(kinectSensorChooser1.Kinect);
            //System.Windows.MessageBox.Show("sjldf");
            if ((t.ThreadState == ThreadState.Running))
            {
                t.Abort();
            }
            //this.Close();
        }




        private void SignalReceived(object sender, SerialDataReceivedEventArgs e)
        {
            //signalsForm.AddText(serialPort.ReadExisting());
            //System.Windows.MessageBox.Show("Got some signal" + e.ToString());
            ack.Set();
        }

        private void ErrorReceived(object sender, SerialErrorReceivedEventArgs e)
        {
            System.Windows.MessageBox.Show("Received error:");
            
        }
    }
}