using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;

namespace Webcam_Test
{
	/// <summary>
	/// Summary description for Form1.
	/// </summary>
	public class Form1 : System.Windows.Forms.Form
	{
		private WebCam_Capture.WebCamCapture UserControl1;
		private WebCam_Capture.WebCamCapture WebCamCapture;
		private System.Windows.Forms.PictureBox pictureBox1;
		private System.Windows.Forms.Button cmdStart;
		private System.Windows.Forms.Button cmdStop;
		private System.Windows.Forms.Button cmdContinue;
		private System.Windows.Forms.NumericUpDown numCaptureTime;
		private System.Windows.Forms.Label label1;
		/// <summary>
		/// Required designer variable.
		/// </summary>
		private System.ComponentModel.Container components = null;

		public Form1()
		{
			//
			// Required for Windows Form Designer support
			//
			InitializeComponent();

			//
			// TODO: Add any constructor code after InitializeComponent call
			//
		}

		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		protected override void Dispose( bool disposing )
		{
			if( disposing )
			{
				if (components != null) 
				{
					components.Dispose();
				}
			}
			base.Dispose( disposing );
		}

		#region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		private void InitializeComponent()
		{
			this.WebCamCapture = new WebCam_Capture.WebCamCapture();
			this.pictureBox1 = new System.Windows.Forms.PictureBox();
			this.cmdStart = new System.Windows.Forms.Button();
			this.cmdStop = new System.Windows.Forms.Button();
			this.cmdContinue = new System.Windows.Forms.Button();
			this.numCaptureTime = new System.Windows.Forms.NumericUpDown();
			this.label1 = new System.Windows.Forms.Label();
			((System.ComponentModel.ISupportInitialize)(this.numCaptureTime)).BeginInit();
			this.SuspendLayout();
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
			this.pictureBox1.Location = new System.Drawing.Point(6, 6);
			this.pictureBox1.Name = "pictureBox1";
			this.pictureBox1.Size = new System.Drawing.Size(320, 240);
			this.pictureBox1.TabIndex = 0;
			this.pictureBox1.TabStop = false;
			// 
			// cmdStart
			// 
			this.cmdStart.Location = new System.Drawing.Point(6, 264);
			this.cmdStart.Name = "cmdStart";
			this.cmdStart.Size = new System.Drawing.Size(78, 24);
			this.cmdStart.TabIndex = 1;
			this.cmdStart.Text = "Start";
			this.cmdStart.Click += new System.EventHandler(this.cmdStart_Click);
			// 
			// cmdStop
			// 
			this.cmdStop.Location = new System.Drawing.Point(102, 264);
			this.cmdStop.Name = "cmdStop";
			this.cmdStop.Size = new System.Drawing.Size(78, 24);
			this.cmdStop.TabIndex = 2;
			this.cmdStop.Text = "Stop";
			this.cmdStop.Click += new System.EventHandler(this.cmdStop_Click);
			// 
			// cmdContinue
			// 
			this.cmdContinue.Location = new System.Drawing.Point(198, 264);
			this.cmdContinue.Name = "cmdContinue";
			this.cmdContinue.Size = new System.Drawing.Size(78, 24);
			this.cmdContinue.TabIndex = 3;
			this.cmdContinue.Text = "Continue";
			this.cmdContinue.Click += new System.EventHandler(this.cmdContinue_Click);
			// 
			// numCaptureTime
			// 
			this.numCaptureTime.Location = new System.Drawing.Point(162, 306);
			this.numCaptureTime.Maximum = new System.Decimal(new int[] {
																		   32000,
																		   0,
																		   0,
																		   0});
			this.numCaptureTime.Minimum = new System.Decimal(new int[] {
																		   1,
																		   0,
																		   0,
																		   0});
			this.numCaptureTime.Name = "numCaptureTime";
			this.numCaptureTime.Size = new System.Drawing.Size(66, 20);
			this.numCaptureTime.TabIndex = 4;
			this.numCaptureTime.Value = new System.Decimal(new int[] {
																		 20,
																		 0,
																		 0,
																		 0});
			// 
			// label1
			// 
			this.label1.Location = new System.Drawing.Point(6, 312);
			this.label1.Name = "label1";
			this.label1.Size = new System.Drawing.Size(150, 18);
			this.label1.TabIndex = 5;
			this.label1.Text = "Capture Time (Milliseconds)";
			// 
			// Form1
			// 
			this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
			this.ClientSize = new System.Drawing.Size(358, 357);
			this.Controls.AddRange(new System.Windows.Forms.Control[] {
																		  this.label1,
																		  this.numCaptureTime,
																		  this.cmdContinue,
																		  this.cmdStop,
																		  this.cmdStart,
																		  this.pictureBox1});
			this.Name = "Form1";
			this.Text = "WebCam Capture";
			this.Closing += new System.ComponentModel.CancelEventHandler(this.Form1_Closing);
			this.Load += new System.EventHandler(this.Form1_Load);
			((System.ComponentModel.ISupportInitialize)(this.numCaptureTime)).EndInit();
			this.ResumeLayout(false);

		}
		#endregion

		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		[STAThread]
		static void Main() 
		{
			Application.Run(new Form1());
		}

		private void Form1_Load(object sender, System.EventArgs e)
		{
			// set the image capture size
			this.WebCamCapture.CaptureHeight = this.pictureBox1.Height;
			this.WebCamCapture.CaptureWidth = this.pictureBox1.Width;
		}

		private void Form1_Closing(object sender, System.ComponentModel.CancelEventArgs e)
		{
			// stop the video capture
			this.WebCamCapture.Stop();
		}

		/// <summary>
		/// An image was capture
		/// </summary>
		/// <param name="source">control raising the event</param>
		/// <param name="e">WebCamEventArgs</param>
		private void WebCamCapture_ImageCaptured(object source, WebCam_Capture.WebcamEventArgs e)
		{
			// set the picturebox picture
			this.pictureBox1.Image = e.WebCamImage;
		}

		private void cmdStart_Click(object sender, System.EventArgs e)
		{
			// change the capture time frame
			this.WebCamCapture.TimeToCapture_milliseconds = (int) this.numCaptureTime.Value;

			// start the video capture. let the control handle the
			// frame numbers.
			this.WebCamCapture.Start(0);

		}

		private void cmdStop_Click(object sender, System.EventArgs e)
		{
			// stop the video capture
			this.WebCamCapture.Stop();
		}

		private void cmdContinue_Click(object sender, System.EventArgs e)
		{
			// change the capture time frame
			this.WebCamCapture.TimeToCapture_milliseconds = (int) this.numCaptureTime.Value;

			// resume the video capture from the stop
			this.WebCamCapture.Start(this.WebCamCapture.FrameNumber);
		}
	}
}
