using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;

namespace HardwareTest
{
    public partial class Form1 : Form
    {
        private Bipedal5Link.ServoController S;

        public Form1()
        {
            InitializeComponent();
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            S = new Bipedal5Link.ServoController("COM1");
            S.Start();
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            S.SetPosition(0, (byte)trackBar1.Value);
            label1.Text = trackBar1.Value.ToString();
        }

        private void trackBar2_Scroll(object sender, EventArgs e)
        {
            S.SetPosition(1, (byte)trackBar2.Value);
            label2.Text = trackBar2.Value.ToString();

        }

        private void trackBar3_Scroll(object sender, EventArgs e)
        {
            S.SetPosition(2, (byte)trackBar3.Value);
            label3.Text = trackBar3.Value.ToString();

        }

        private void trackBar4_Scroll(object sender, EventArgs e)
        {
            S.SetPosition(3, (byte)trackBar4.Value);
            label4.Text = trackBar4.Value.ToString();

        }

    }

}