using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using gbp.AI.GeneticAlgorithms;

namespace Bipedal5LinkTest
{
    public partial class Form1 : Form
    {
        private Bipedal5Link.Bipedal5Link B;
        private Bipedal5Link.CentralPatternGenerator CPG;
        private bool Abort;

        private Bitmap DisplayBuffer;
        private Graphics DisplayBufferG;
        private Graphics LabelDisplayG;
        private Bitmap StatusBuffer;
        private Graphics StatusBufferG;
        private Graphics LabelStatusG;
        private Graphics LabelStatus2G;
        private Bitmap CPGBuffer;
        private Graphics CPGBufferG;
        private Graphics LabelCPGG;
        private Color C1;
        private Font F1;
        private double PiOverTwo;

        private Bipedal5Link.ServoController Hardware;

        private gbp.AI.GeneticAlgorithms.GeneticAlgorithms GA;

        public Form1()
        {
            InitializeComponent();

            DisplayBuffer = new Bitmap(1260, 350);
            DisplayBufferG = Graphics.FromImage(DisplayBuffer);
            DisplayBufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
            LabelDisplayG = labelDisplay.CreateGraphics();
            LabelDisplayG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

            StatusBuffer = new Bitmap(950, 170);
            StatusBufferG = Graphics.FromImage(StatusBuffer);
            //StatusBufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
            LabelStatusG = labelStatus.CreateGraphics();
            LabelStatus2G = labelStatus2.CreateGraphics();
            //LabelStatusG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

            CPGBuffer = new Bitmap(300, 240);
            CPGBufferG = Graphics.FromImage(CPGBuffer);
            CPGBufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
            LabelCPGG = labelCPG.CreateGraphics();
            LabelCPGG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

            NewBipedal5Link();
            NewCentralPatternGenerator();
            C1 = Color.FromArgb(35, 22, 0);
            F1 = new Font("Arial", 8);

            PiOverTwo = Math.PI / 2;

            Hardware = new Bipedal5Link.ServoController("COM1");
        }

        private double YGroundFlat(double x)
        {
            return 0;
            //return x / 80;
        }

        private void NewBipedal5Link()
        {
            double m = double.Parse(textBoxMsM.Text);
            double m1 = double.Parse(textBoxMsM1.Text);
            double l1 = double.Parse(textBoxMsL1.Text);
            double m2 = double.Parse(textBoxMsM2.Text);
            double l2 = double.Parse(textBoxMsL2.Text);
            double b1 = double.Parse(textBoxMsB1.Text);
            double b2 = double.Parse(textBoxMsB2.Text);
            double kk = double.Parse(textBoxMsKk.Text);
            double bk = double.Parse(textBoxMsBk.Text);
            double kg = double.Parse(textBoxMsKg.Text);
            double bg = double.Parse(textBoxMsBg.Text);
            double ss = double.Parse(textBoxMsSS.Text);
            double g = double.Parse(textBoxMsG.Text);
            double hipx = double.Parse(textBoxMsHipX.Text);
            double hipy = double.Parse(textBoxMsHipY.Text);
            double theta = double.Parse(textBoxMsTheta.Text);
            double theta1 = double.Parse(textBoxMsTheta1.Text);
            double theta2 = double.Parse(textBoxMsTheta2.Text);
            double theta3 = double.Parse(textBoxMsTheta3.Text);
            double theta4 = double.Parse(textBoxMsTheta4.Text);

            B = new Bipedal5Link.Bipedal5Link(m, m1, m2, l1, l2, kg, bg, g, new Bipedal5Link.PointD(hipx, hipy), new double[] {theta, theta1, theta2, theta3, theta4}, ss, new Bipedal5Link.Bipedal5Link.YGroundDelegate(YGroundFlat));
            //B.RunStep(0);

            DisplayBufferG.Clear(C1);
            B.Draw(DisplayBufferG, new PointF(10, 210), 500);
            LabelDisplayG.DrawImage(DisplayBuffer, 0, 0);
        }

        private void NewCentralPatternGenerator()
        {
            double hipTauU = double.Parse(textBoxCpgHipTauU.Text);
            double hipTauV = double.Parse(textBoxCpgHipTauV.Text);
            double hipBeta = double.Parse(textBoxCpgHipBeta.Text);
            double kneeTauU = double.Parse(textBoxCpgKneeTauU.Text);
            double kneeTauV = double.Parse(textBoxCpgKneeTauV.Text);
            double kneeBeta = double.Parse(textBoxCpgKneeBeta.Text);
            double wfe = double.Parse(textBoxCpgWFE.Text);
            double w0 = double.Parse(textBoxCpgW1.Text);
            double w1 = double.Parse(textBoxCpgW2.Text);
            double w2 = double.Parse(textBoxCpgW3.Text);
            double w3 = double.Parse(textBoxCpgW4.Text);
            double w4 = double.Parse(textBoxCpgW5.Text);
            double w5 = double.Parse(textBoxCpgW6.Text);
            double w6 = double.Parse(textBoxCpgW7.Text);
            double w7 = double.Parse(textBoxCpgW8.Text);

            double[,] weights = new double[9, 9];
            weights[1, 2] = wfe;
            weights[1, 3] = w0;
            weights[1, 4] = w1;
            weights[2, 1] = wfe;
            weights[2, 3] = w2;
            weights[2, 4] = w3;
            weights[3, 1] = w0;
            weights[3, 2] = w1;
            weights[3, 4] = wfe;
            weights[4, 1] = w2;
            weights[4, 2] = w3;
            weights[4, 3] = wfe;
            weights[5, 1] = w4;
            weights[5, 2] = w5;
            weights[5, 6] = wfe;
            weights[6, 1] = w6;
            weights[6, 2] = w7;
            weights[6, 5] = wfe;
            weights[7, 3] = w4;
            weights[7, 4] = w5;
            weights[7, 8] = wfe;
            weights[8, 3] = w6;
            weights[8, 4] = w7;
            weights[8, 7] = wfe;

            CPG = new Bipedal5Link.CentralPatternGenerator(hipTauU, hipTauV, hipBeta, kneeTauU, kneeTauV, kneeBeta, weights);
        }

        private void linkLabelNew_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            NewBipedal5Link();
        }
        private void linkLabelRun_LinkClickedOriginal(object sender, LinkLabelLinkClickedEventArgs e)
        {
            NewBipedal5Link();

            NewCentralPatternGenerator();

            timerUnstable.Stop();
            panelUnstable.Hide();

            int steps = int.Parse(textBoxSteps.Text);
            double deltatime = double.Parse(textBoxDeltaTime.Text);
            double u0 = double.Parse(textBoxCpgU0.Text);
            double a1 = double.Parse(textBoxCouplingA1.Text);
            double a2 = double.Parse(textBoxCouplingA2.Text);
            int hipfixsteps = int.Parse(textBoxMsHipFixSteps.Text);

            StatusBufferG.Clear(Color.Black);
            StatusBufferG.DrawString(B.Parameters(), F1, Brushes.DarkOrange, 0, 10);
            StatusBufferG.DrawString(CPG.Parameters(), F1, Brushes.DarkOrange, 440, 10);

            double[] joints;
            double[] feedback = new double[9];
            double f1;
            double f2;
            double f3;
            double f4;
            double f5;
            double f6;
            double f7;
            double f8;

            for (int s = 0 ; s < 100000 ; s++)
                joints = CPG.RunStep(u0, feedback, deltatime);

            B.HipFixed = true;
            Abort = false;

            // time coupling
            double simulationtime = 0;
            DateTime realtimestart = DateTime.Now;
            TimeSpan realtime;

            DisplayBufferG.Clear(Color.White);

            for (int s = 0 ; (s < steps) & !Abort ; s++)
            {
                // Run CPG step
                f1 = -a1 * feedback[1] + a1 * feedback[2] + a1 * feedback[6];
                f2 = -f1;
                f3 = -a1 * feedback[2] + a1 * feedback[1] + a1 * feedback[5];
                f4 = -f3;
                f5 = a2 * feedback[6] * feedback[4];
                f6 = -f5;
                f7 = a2 * feedback[5] * feedback[3];
                f8 = -f7;
                joints = CPG.RunStep(u0, new double[] { 0, f1, f2, f3, f4, f5, f6, f7, f8 }, deltatime);

                // Run Musculo-Skeletal system step
                if (s > hipfixsteps)
                    B.HipFixed = false;

                feedback = B.RunStep(joints, deltatime);

                if (!B.IsStable())
                    timerUnstable.Start();

                simulationtime = s * deltatime;
                realtime = DateTime.Now.Subtract(realtimestart);
                if (simulationtime > realtime.TotalSeconds)
                {
                    //DisplayBufferG.Clear(C1);
                    B.Draw(DisplayBufferG, new PointF(10, 240), 500);
                    LabelDisplayG.DrawImage(DisplayBuffer, 0, 0);

                    StatusBufferG.FillRectangle(Brushes.Black, 240, 0, 200, 160);
                    StatusBufferG.DrawString(B.Status(), F1, Brushes.DarkOrange, 240, 10);
                    StatusBufferG.FillRectangle(Brushes.Black, 640, 0, 200, 160);
                    StatusBufferG.DrawString(CPG.Status(), F1, Brushes.DarkOrange, 640, 10);
                    LabelStatusG.DrawImage(StatusBuffer, 0, 0);

                    CPGBufferG.Clear(Color.Black);
                    CPG.Draw(CPGBufferG);
                    LabelCPGG.DrawImage(CPGBuffer, 0, 0);

                    System.Windows.Forms.Application.DoEvents();
                }
            }
        }

        private void linkLabelRun_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            NewBipedal5Link();

            NewCentralPatternGenerator();

            timerUnstable.Stop();
            panelUnstable.Hide();

            int steps = int.Parse(textBoxSteps.Text);
            double deltatime = double.Parse(textBoxDeltaTime.Text);
            double u0 = double.Parse(textBoxCpgU0.Text);
            double a1 = double.Parse(textBoxCouplingA1.Text);
            double a2 = double.Parse(textBoxCouplingA2.Text);
            int hipfixsteps = int.Parse(textBoxMsHipFixSteps.Text);

            StatusBufferG.Clear(Color.Black);
            StatusBufferG.DrawString(B.Parameters(), F1, Brushes.DarkOrange, 0, 10);
            StatusBufferG.DrawString(CPG.Parameters(), F1, Brushes.DarkOrange, 440, 10);

            double[] joints;
            double[] feedback = new double[9];
            double f1;
            double f2;
            double f3;
            double f4;
            double f5;
            double f6;
            double f7;
            double f8;

            for (int s = 0 ; s < 100000 ; s++)
                joints = CPG.RunStep(u0, feedback, deltatime);

            B.HipFixed = true;
            Abort = false;

            // time coupling
            double simulationtime = 0;
            DateTime realtimestart = DateTime.Now;
            TimeSpan realtime;
            
            DisplayBufferG.Clear(Color.White);

            for (int s = 0 ; (s < steps) & !Abort ; s++)
            {
                // Run CPG step
                f1 = -a1 * feedback[1] + a1 * feedback[2] + a1 * feedback[6];
                f2 = -f1;
                f3 = -a1 * feedback[2] + a1 * feedback[1] + a1 * feedback[5];
                f4 = -f3;
                f5 = a2 * feedback[6] * feedback[4];
                f6 = -f5;
                f7 = a2 * feedback[5] * feedback[3];
                f8 = -f7;
                joints = CPG.RunStep(u0, new double[] { 0, f1, f2, f3, f4, f5, f6, f7, f8 }, deltatime);

                // Run Musculo-Skeletal system step
                if (s > hipfixsteps)
                    B.HipFixed = false;

                feedback = B.RunStep(joints, deltatime);

                if (!B.IsStable())
                    timerUnstable.Start();

                simulationtime = s * deltatime;
                realtime = DateTime.Now.Subtract(realtimestart);
                //if (s % 10000 == 0)
                if (simulationtime > realtime.TotalSeconds)
                {
                    DisplayBufferG.Clear(C1);
                    B.Draw(DisplayBufferG, new PointF(10, 210), 500);
                    LabelDisplayG.DrawImage(DisplayBuffer, 0, 0);

                    StatusBufferG.FillRectangle(Brushes.Black, 240, 0, 200, 160);
                    StatusBufferG.DrawString(B.Status(), F1, Brushes.DarkOrange, 240, 10);
                    StatusBufferG.FillRectangle(Brushes.Black, 640, 0, 200, 160);
                    StatusBufferG.DrawString(CPG.Status(), F1, Brushes.DarkOrange, 640, 10);
                    LabelStatusG.DrawImage(StatusBuffer, 0, 0);

                    CPGBufferG.Clear(Color.Black);
                    CPG.Draw(CPGBufferG);
                    LabelCPGG.DrawImage(CPGBuffer, 0, 0);

                    System.Windows.Forms.Application.DoEvents();
                }
            }
        }

        private void linkLabelStop_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Abort = true;
            timerUnstable.Stop();
            panelUnstable.Hide();
        }

        private void labelDisplay_Paint(object sender, PaintEventArgs e)
        {
            e.Graphics.DrawImage(DisplayBuffer, 0, 0);
        }

        private void linkLabelExit_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Abort = true;
            if (Hardware.Running)
                Hardware.Stop();
            this.Close();
        }

        private void labelStatus_Paint(object sender, PaintEventArgs e)
        {
            e.Graphics.DrawImage(StatusBuffer, 0, 0);
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            NewCentralPatternGenerator();
        }

        private void timerUnstable_Tick(object sender, EventArgs e)
        {
            panelUnstable.Visible = !panelUnstable.Visible;
        }

        private void labelCPG_Paint(object sender, PaintEventArgs e)
        {
            e.Graphics.DrawImage(CPGBuffer, 0, 0);
        }

        private void textBoxMsTheta1_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBoxMsTheta3_TextChanged(object sender, EventArgs e)
        {

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            labelVersion.Text = "v1.0.4";
            CheckForIllegalCrossThreadCalls = false;
        }

        private void linkLabelRunGA_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            int steps = int.Parse(textBoxSteps.Text);
            double deltatime = double.Parse(textBoxDeltaTime.Text);
            int hipfixsteps = int.Parse(textBoxMsHipFixSteps.Text);

            int size = int.Parse(textBoxGASize.Text);
            int generations = int.Parse(textBoxGAGenerations.Text);
            float fitnesstreshold = float.Parse(textBoxGAFitnessTreshold.Text);
            float crossover = float.Parse(textBoxGACrossover.Text);
            float mutation = float.Parse(textBoxGAMutation.Text);
            bool elitism = checkBoxGAElitism.Checked;
            int tournamentsize = int.Parse(textBoxGATournamentSize.Text);
            float tournamentchance = float.Parse(textBoxGATournamentChance.Text);

            double m = double.Parse(textBoxMsM.Text);
            double m1 = double.Parse(textBoxMsM1.Text);
            double l1 = double.Parse(textBoxMsL1.Text);
            double m2 = double.Parse(textBoxMsM2.Text);
            double l2 = double.Parse(textBoxMsL2.Text);
            double kg = double.Parse(textBoxMsKg.Text);
            double bg = double.Parse(textBoxMsBg.Text);
            double ss = double.Parse(textBoxMsSS.Text);
            double g = double.Parse(textBoxMsG.Text);
            double hipx = double.Parse(textBoxMsHipX.Text);
            double hipy = double.Parse(textBoxMsHipY.Text);
            double theta = double.Parse(textBoxMsTheta.Text);
            double theta1 = double.Parse(textBoxMsTheta1.Text);
            double theta2 = double.Parse(textBoxMsTheta2.Text);
            double theta3 = double.Parse(textBoxMsTheta3.Text);
            double theta4 = double.Parse(textBoxMsTheta4.Text);

            GA = new GABipedal5Link(size, 22, 0f, 1f, generations, crossover, mutation, tournamentsize, tournamentchance, elitism, fitnesstreshold, new SolutionFoundDelegate(GASolutionFound), new StoppedDelegate(GAStopped), new ImprovementDelegate(Improvement), LabelStatusG, LabelCPGG, LabelStatus2G, LabelDisplayG, Color.Black, Color.Orange, Color.DarkOrange, steps, deltatime, hipfixsteps, m, m1, m2, l1, l2, kg, bg, g, new Bipedal5Link.PointD(hipx, hipy), new double[] { theta, theta1, theta2, theta3, theta4 }, ss, new Bipedal5Link.Bipedal5Link.YGroundDelegate(YGroundFlat));
            GA.ShowEvaluation = checkBoxGAEvaluation.Checked;
            GA.Evolve();
        }

        private void Improvement(string text)
        {
            textBoxImprovement.Text += text;
        }

        private void GASolutionFound(float[] genome, int generation)
        {
            double w0 = -genome[0] * 2;
            double w1 = -genome[1] * 2;
            double w2 = -genome[2] * 2;
            double w3 = -genome[3] * 2;

            double w4 = -genome[4] * 2;
            double w5 = -genome[5] * 2;
            double w6 = -genome[6] * 2;
            double w7 = -genome[7] * 2;

            double x0 = genome[8] > 0.5 ? 1 : 0;
            double x1 = genome[9] > 0.5 ? 1 : 0;
            double x2 = genome[10] > 0.5 ? 1 : 0;
            double x3 = genome[11] > 0.5 ? 1 : 0;

            double x4 = genome[12] > 0.5 ? 1 : 0;
            double x5 = genome[13] > 0.5 ? 1 : 0;
            double x6 = genome[14] > 0.5 ? 1 : 0;
            double x7 = genome[15] > 0.5 ? 1 : 0;

            w0 = w0 * x0;
            w1 = w1 * x1;
            w2 = w2 * x2;
            w3 = w3 * x3;
            w4 = w4 * x4;
            w5 = w5 * x5;
            w6 = w6 * x6;
            w7 = w7 * x7;

            double wfe = -2;
            double u0 = genome[16] * 5;
            double hipTauU = genome[17] / 2;
            double hipTauV = genome[18] / 2;
            double hipBeta = genome[19] * 5;
            double kneeTauU = hipTauU / 2;
            double kneeTauV = hipTauV / 2;
            double kneeBeta = hipBeta;
            double a1 = genome[20] * 2;
            double a2 = genome[21] * 2;

            double[,] weights = new double[9, 9];
            weights[1, 2] = wfe;
            weights[1, 3] = w0;
            weights[1, 4] = w1;
            weights[2, 1] = wfe;
            weights[2, 3] = w2;
            weights[2, 4] = w3;
            weights[3, 1] = w0;
            weights[3, 2] = w1;
            weights[3, 4] = wfe;
            weights[4, 1] = w2;
            weights[4, 2] = w3;
            weights[4, 3] = wfe;
            weights[5, 1] = w4;
            weights[5, 2] = w5;
            weights[5, 6] = wfe;
            weights[6, 1] = w6;
            weights[6, 2] = w7;
            weights[6, 5] = wfe;
            weights[7, 3] = w4;
            weights[7, 4] = w5;
            weights[7, 8] = wfe;
            weights[8, 3] = w6;
            weights[8, 4] = w7;
            weights[8, 7] = wfe;
            
            textBoxCpgHipTauU.Text = hipTauU.ToString("f3");
            textBoxCpgHipTauV.Text = hipTauV.ToString("f3");
            textBoxCpgHipBeta.Text = hipBeta.ToString("f3");
            textBoxCpgKneeTauU.Text = kneeTauU.ToString("f3");
            textBoxCpgKneeTauV.Text = kneeTauV.ToString("f3");
            textBoxCpgKneeBeta.Text = kneeBeta.ToString("f3");
            textBoxCpgU0.Text = u0.ToString("f3");
            textBoxCpgWFE.Text = wfe.ToString("f3");
            textBoxCpgW1.Text = w0.ToString("f3");
            textBoxCpgW2.Text = w1.ToString("f3");
            textBoxCpgW3.Text = w2.ToString("f3");
            textBoxCpgW4.Text = w3.ToString("f3");
            textBoxCpgW5.Text = w4.ToString("f3");
            textBoxCpgW6.Text = w5.ToString("f3");
            textBoxCpgW7.Text = w6.ToString("f3");
            textBoxCpgW8.Text = w7.ToString("f3");
            textBoxCouplingA1.Text = a1.ToString("f3");
            textBoxCouplingA2.Text = a2.ToString("f3");

        }

        private void GASolutionFound2(float[] solution, int generation)
        {
            //double w0 = -solution[0] * 2;
            //double w1 = -solution[1] * 2;
            //double w2 = -solution[2] * 2;
            //double w3 = -solution[3] * 2;

            //double w4 = -solution[4] * 2;
            //double w5 = -solution[5] * 2;
            //double w6 = -solution[6] * 2;
            //double w7 = -solution[7] * 2;

            //double x0 = solution[8] > 0.5 ? 1 : 0;
            //double x1 = solution[9] > 0.5 ? 1 : 0;
            //double x2 = solution[10] > 0.5 ? 1 : 0;
            //double x3 = solution[11] > 0.5 ? 1 : 0;

            //double x4 = solution[12] > 0.5 ? 1 : 0;
            //double x5 = solution[13] > 0.5 ? 1 : 0;
            //double x6 = solution[14] > 0.5 ? 1 : 0;
            //double x7 = solution[15] > 0.5 ? 1 : 0;

            double w0 = -1;
            double w1 = 0;
            double w2 = 0;
            double w3 = -1;

            double w4 = 0;
            double w5 = -2;
            double w6 = -2;
            double w7 = -0;

            double x0 = 1;
            double x1 = 0;
            double x2 = 0;
            double x3 = 1;

            double x4 = 0;
            double x5 = 1;
            double x6 = 1;
            double x7 = 0;

            double wfe = -2;

            double u0 = solution[0] * 5;
            double hipTauU = solution[1] / 10;
            double hipTauV = solution[2] / 10;
            double hipBeta = solution[3] * 5;
            double kneeTauU = hipTauU / 2;
            double kneeTauV = hipTauV / 2;
            double kneeBeta = hipBeta;
            w0 = w0 * x0;
            w1 = w1 * x1;
            w2 = w2 * x2;
            w3 = w3 * x3;
            w4 = w4 * x4;
            w5 = w5 * x5;
            w6 = w6 * x6;
            w7 = w7 * x7;

            textBoxCpgHipTauU.Text = hipTauU.ToString("f3");
            textBoxCpgHipTauV.Text = hipTauV.ToString("f3");
            textBoxCpgHipBeta.Text = hipBeta.ToString("f3");
            textBoxCpgKneeTauU.Text = kneeTauU.ToString("f3");
            textBoxCpgKneeTauV.Text = kneeTauV.ToString("f3");
            textBoxCpgKneeBeta.Text = kneeBeta.ToString("f3");
            textBoxCpgU0.Text = u0.ToString("f3");
            textBoxCpgWFE.Text = wfe.ToString("f3");
            textBoxCpgW1.Text = w0.ToString("f3");
            textBoxCpgW2.Text = w1.ToString("f3");
            textBoxCpgW3.Text = w2.ToString("f3");
            textBoxCpgW4.Text = w3.ToString("f3");
            textBoxCpgW5.Text = w4.ToString("f3");
            textBoxCpgW6.Text = w5.ToString("f3");
            textBoxCpgW7.Text = w6.ToString("f3");
            textBoxCpgW8.Text = w7.ToString("f3");

        }

        private void GAStopped()
        {
            labelGAStatus.Text = ".";
        }

        private class GABipedal5Link : gbp.AI.GeneticAlgorithms.GeneticAlgorithms
        {
            private int Steps;
            private double DeltaTime;
            private int HipFixSteps;

            private double M;
            private double M1;
            private double M2;
            private double G;
            private double KG;
            private double BG;
            private double L1;
            private double L2;
            private double ServoSpeed;
            private Bipedal5Link.Bipedal5Link.YGroundDelegate YGround;
            private Bipedal5Link.PointD Hip;
            private double[] Theta;

            private Color C1;
            private Font F1;

            public GABipedal5Link(int size, int chromosomeLength, float geneMinimum, float geneMaximum, int generations, float crossoverChance, float mutationChance, int tournamentSize, float tournamentChance, bool elitism, float fitnessTreshold, SolutionFoundDelegate sf, StoppedDelegate stp, ImprovementDelegate imp, Graphics gReport, Graphics gReport2, Graphics gReport3, Graphics gFitness, Color backColor, Color foreColor1, Color foreColor2, int steps, double deltaTime, int hipFixSteps, double m, double m1, double m2, double l1, double l2, double kg, double bg, double g, Bipedal5Link.PointD hip, double[] theta, double servoSpeed, Bipedal5Link.Bipedal5Link.YGroundDelegate yGround)
                : base(size, chromosomeLength, geneMinimum, geneMaximum, generations, crossoverChance, mutationChance, tournamentSize, tournamentChance, elitism, fitnessTreshold, sf, stp, imp, gReport, gReport2, gReport3, gFitness, backColor, foreColor1, foreColor2)
            {
                Steps = steps;
                DeltaTime = deltaTime;
                HipFixSteps = hipFixSteps;

                M = m;
                M1 = m1;
                M2 = m2;
                G = g;
                KG = kg;
                BG = bg;
                L1 = l1;
                L2 = l2;
                ServoSpeed = servoSpeed;
                YGround = yGround;
                Hip = hip;
                Theta = theta;

                IndividualsDrawn = false;
                C1 = Color.FromArgb(35, 22, 0);
                F1 = new Font("Arial", 8);
            }

            public override void DrawIndividual(float[] individual, Graphics g, RectangleF position)
            {
            }

            public float Fitness2(float[] individual, int index, bool showEvaluation)
            {
                //DrawIndividual(individual, GFitnessBufferG, new RectangleF(0, 0, 200, 100));

                double wfe = -2;
                double wlr = -1;
                double whk = -1;

                double u0 = individual[0] * 5;
                double hipTauU = individual[1] / 10;
                double hipTauV = individual[2] / 10;
                double hipBeta = individual[3] * 5;
                double kneeTauU = hipTauU / 2;
                double kneeTauV = hipTauV / 2;
                double kneeBeta = hipBeta;

               
                string parameters = "u0 = " + u0.ToString("f2") + ", hTauU = " + hipTauU.ToString("f2") + ", hTauV = " + hipTauV.ToString("f2") + ", hBeta = " + hipBeta.ToString("f2") + ", kTauU = " + kneeTauU.ToString("f2") + ", kTauV = " + kneeTauV.ToString("f2") + ", kBeta = " + kneeBeta.ToString("f2");

                Bipedal5Link.CentralPatternGenerator cpg = new Bipedal5Link.CentralPatternGenerator(hipTauU, hipTauV, hipBeta, kneeTauU, kneeTauV, kneeBeta, wfe, whk, wlr);
                Bipedal5Link.Bipedal5Link b = new Bipedal5Link.Bipedal5Link(M, M1, M2, L1, L2, KG, BG, G, Hip, Theta, ServoSpeed, YGround);

                int update = (int)(0.005 / DeltaTime);

                double[] joints;
                double[] feedback = new double[9];

                for (int s = 0 ; s < 100000 ; s++)
                    joints = cpg.RunStep(u0, feedback, DeltaTime);

                b.HipFixed = true;
                for (int s = 0 ; s < Steps ; s++)
                {
                    // Run CPG step
                    joints = cpg.RunStep(u0, new double[] { 0, feedback[1], -feedback[1], feedback[2], -feedback[2], feedback[3], -feedback[3], feedback[4], -feedback[4] }, DeltaTime);

                    // Run Musculo-Skeletal system step
                    if (s > HipFixSteps)
                        b.HipFixed = false;

                    feedback = b.RunStep(joints, DeltaTime);
                    if (!b.IsStable())
                        break;

                    if (showEvaluation && (s % update == 0))
                    {
                        GFitnessBufferG.Clear(C1);
                        b.Draw(GFitnessBufferG, new PointF(10, 210), 500);
                        GFitnessBufferG.DrawString("individual " + index.ToString() + ": " + parameters, F1, Brushes.Orange, 0, 0);
                        GFitness.DrawImage(GFitnessBuffer, 0, 0);

                        //StatusBufferG.FillRectangle(Brushes.Black, 240, 0, 200, 160);
                        //StatusBufferG.DrawString(B.Status(), F1, Brushes.DarkOrange, 240, 10);
                        //StatusBufferG.FillRectangle(Brushes.Black, 640, 0, 200, 160);
                        //StatusBufferG.DrawString(CPG.Status(), F1, Brushes.DarkOrange, 640, 10);
                        //LabelStatusG.DrawImage(StatusBuffer, 0, 0);

                        GReport2BufferG.Clear(Color.Black);
                        cpg.Draw(GReport2BufferG);
                        GReport2.DrawImage(GReport2Buffer, 0, 0);
                    }
                }

                return (float)Math.Max(b.BehindPosition().X, 0.001);

            }

            public override float Fitness(float[] genome, int generation, int index, bool showEvaluation)
            {
                //DrawIndividual(individual, GFitnessBufferG, new RectangleF(0, 0, 200, 100));

                double w0 = -genome[0] * 2;
                double w1 = -genome[1] * 2;
                double w2 = -genome[2] * 2;
                double w3 = -genome[3] * 2;

                double w4 = -genome[4] * 2;
                double w5 = -genome[5] * 2;
                double w6 = -genome[6] * 2;
                double w7 = -genome[7] * 2;

                double x0 = genome[8] > 0.5 ? 1 : 0;
                double x1 = genome[9] > 0.5 ? 1 : 0;
                double x2 = genome[10] > 0.5 ? 1 : 0;
                double x3 = genome[11] > 0.5 ? 1 : 0;

                double x4 = genome[12] > 0.5 ? 1 : 0;
                double x5 = genome[13] > 0.5 ? 1 : 0;
                double x6 = genome[14] > 0.5 ? 1 : 0;
                double x7 = genome[15] > 0.5 ? 1 : 0;

                w0 = w0 * x0;
                w1 = w1 * x1;
                w2 = w2 * x2;
                w3 = w3 * x3;
                w4 = w4 * x4;
                w5 = w5 * x5;
                w6 = w6 * x6;
                w7 = w7 * x7;

                double wfe = -2;
                double u0 = genome[16] * 5;
                double hipTauU = genome[17] / 2;
                double hipTauV = genome[18] / 2;
                double hipBeta = genome[19] * 5;
                double kneeTauU = hipTauU / 2;
                double kneeTauV = hipTauV / 2;
                double kneeBeta = hipBeta;

                double a1 = genome[20] * 2;
                double a2 = genome[21] * 2;

                double[,] weights = new double[9, 9];
                weights[1, 2] = wfe;
                weights[1, 3] = w0;
                weights[1, 4] = w1;
                weights[2, 1] = wfe;
                weights[2, 3] = w2;
                weights[2, 4] = w3;
                weights[3, 1] = w0;
                weights[3, 2] = w1;
                weights[3, 4] = wfe;
                weights[4, 1] = w2;
                weights[4, 2] = w3;
                weights[4, 3] = wfe;
                weights[5, 1] = w4;
                weights[5, 2] = w5;
                weights[5, 6] = wfe;
                weights[6, 1] = w6;
                weights[6, 2] = w7;
                weights[6, 5] = wfe;
                weights[7, 3] = w4;
                weights[7, 4] = w5;
                weights[7, 8] = wfe;
                weights[8, 3] = w6;
                weights[8, 4] = w7;
                weights[8, 7] = wfe;

                string parameters = "u0 = " + u0.ToString("f2") + ", hTauU = " + hipTauU.ToString("f2") + ", hTauV = " + hipTauV.ToString("f2") + ", hBeta = " + hipBeta.ToString("f2") + ", kTauU = " + kneeTauU.ToString("f2") + ", kTauV = " + kneeTauV.ToString("f2") + ", kBeta = " + kneeBeta.ToString("f2");

                Bipedal5Link.CentralPatternGenerator cpg = new Bipedal5Link.CentralPatternGenerator(hipTauU, hipTauV, hipBeta, kneeTauU, kneeTauV, kneeBeta, weights);
                Bipedal5Link.Bipedal5Link b = new Bipedal5Link.Bipedal5Link(M, M1, M2, L1, L2, KG, BG, G, Hip, Theta, ServoSpeed, YGround);

                double[] joints;
                double[] feedback = new double[9];
                double f1;
                double f2;
                double f3;
                double f4;
                double f5;
                double f6;
                double f7;
                double f8;


                for (int s = 0 ; s < 100000 ; s++)
                    joints = cpg.RunStep(u0, feedback, DeltaTime);

                b.HipFixed = true;

                // time coupling
                double simulationtime = 0;
                DateTime realtimestart = DateTime.Now;
                TimeSpan realtime;
 
                for (int s = 0 ; s < Steps ; s++)
                {
                    // Run CPG step
                    //f1 = a1 * feedback[1] + a2 * feedback[4];
                    //f2 = -a1 * feedback[1] - a2 * feedback[4];
                    //f3 = a1 * feedback[2] + a2 * feedback[3];
                    //f4 = -a1 * feedback[2] - a2 * feedback[3];
                    f1 = -a1 * feedback[1] + a1 * feedback[2] + a1 * feedback[6];
                    f2 = -f1;
                    f3 = -a1 * feedback[2] + a1 * feedback[1] + a1 * feedback[5];
                    f4 = -f3;
                    f5 = a2 * feedback[6] * feedback[4];
                    f6 = -f5;
                    f7 = a2 * feedback[5] * feedback[3];
                    f8 = -f7;
                    joints = cpg.RunStep(u0, new double[] { 0, f1, f2, f3, f4, f5, f6, f7, f8 }, DeltaTime);

                    // Run Musculo-Skeletal system step
                    if (s > HipFixSteps)
                        b.HipFixed = false;

                    feedback = b.RunStep(joints, DeltaTime);
                    if (!b.IsStable())
                        break;

                    if (showEvaluation)
                    {
                        simulationtime = s * DeltaTime;
                        realtime = DateTime.Now.Subtract(realtimestart);
                        if (simulationtime > realtime.TotalSeconds)
                        {
                            GFitnessBufferG.Clear(C1);
                            b.Draw(GFitnessBufferG, new PointF(10, 210), 500);
                            GFitnessBufferG.DrawString("individual " + index.ToString() + ": " + parameters, F1, Brushes.Orange, 0, 0);
                            GFitness.DrawImage(GFitnessBuffer, 0, 0);

                            //StatusBufferG.FillRectangle(Brushes.Black, 240, 0, 200, 160);
                            //StatusBufferG.DrawString(B.Status(), F1, Brushes.DarkOrange, 240, 10);
                            //StatusBufferG.FillRectangle(Brushes.Black, 640, 0, 200, 160);
                            //StatusBufferG.DrawString(CPG.Status(), F1, Brushes.DarkOrange, 640, 10);
                            //LabelStatusG.DrawImage(StatusBuffer, 0, 0);

                            GReport2BufferG.Clear(Color.Black);
                            cpg.Draw(GReport2BufferG);
                            GReport2.DrawImage(GReport2Buffer, 0, 0);
                        }
                    }
                }

                float fit = (float)Math.Max(b.BehindPosition().X, 0.001);

                if (showEvaluation)
                {
                    StringBuilder sb = new StringBuilder();
                    sb.Append("Fit: ");
                    sb.Append(fit.ToString("f3"));
                    sb.Append("; Gen: ");
                    sb.Append(generation);
                    sb.Append("; Ind: ");
                    sb.Append(index);
                    sb.Append("; a1: ");
                    sb.Append(a1.ToString("f3"));
                    sb.Append("; a2: ");
                    sb.Append(a2.ToString("f3"));
                    sb.Append("; wfe: ");
                    sb.Append(wfe.ToString("f3"));
                    sb.Append("; u0: ");
                    sb.Append(u0.ToString("f3"));
                    sb.Append("; hipTauU: ");
                    sb.Append(hipTauU.ToString("f3"));
                    sb.Append("; hipTauV: ");
                    sb.Append(hipTauV.ToString("f3"));
                    sb.Append("; hipBeta: ");
                    sb.Append(hipBeta.ToString("f3"));
                    sb.Append("; kneeTauU: ");
                    sb.Append(kneeTauU.ToString("f3"));
                    sb.Append("; kneeTauV: ");
                    sb.Append(kneeTauV.ToString("f3"));
                    sb.Append("; kneeBeta: ");
                    sb.Append(kneeBeta.ToString("f3"));
                    sb.Append("; w0: ");
                    sb.Append(w0.ToString("f3"));
                    sb.Append("; w1: ");
                    sb.Append(w1.ToString("f3"));
                    sb.Append("; w2: ");
                    sb.Append(w2.ToString("f3"));
                    sb.Append("; w3: ");
                    sb.Append(w3.ToString("f3"));
                    sb.Append("; w4: ");
                    sb.Append(w4.ToString("f3"));
                    sb.Append("; w5: ");
                    sb.Append(w5.ToString("f3"));
                    sb.Append("; w6: ");
                    sb.Append(w6.ToString("f3"));
                    sb.Append("; w7: ");
                    sb.Append(w7.ToString("f3"));
                    sb.Append(Environment.NewLine);
                    IMP(sb.ToString());
                }

                return fit;

            }

        }

        private void linkLabelStopGA_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            if (GA != null)
            {
                labelGAStatus.Text = "stopping...";
                GA.Stop();
            }
        }

        private void checkBoxGAEvaluation_CheckedChanged(object sender, EventArgs e)
        {
            if (GA != null)
                GA.ShowEvaluation = checkBoxGAEvaluation.Checked;
        }

        private void checkBoxGAImprovement_CheckedChanged(object sender, EventArgs e)
        {
            if (GA != null)
                GA.ShowImprovements = checkBoxGAImprovement.Checked;
        }

        private void linkLabelRunHardware_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            double m = double.Parse(textBoxMsM.Text);
            double m1 = double.Parse(textBoxMsM1.Text);
            double l1 = double.Parse(textBoxMsL1.Text);
            double m2 = double.Parse(textBoxMsM2.Text);
            double l2 = double.Parse(textBoxMsL2.Text);
            double ss = double.Parse(textBoxMsSS.Text);
            double theta = double.Parse(textBoxMsTheta.Text);
            double theta1 = double.Parse(textBoxMsTheta1.Text);
            double theta2 = double.Parse(textBoxMsTheta2.Text);
            double theta3 = double.Parse(textBoxMsTheta3.Text);
            double theta4 = double.Parse(textBoxMsTheta4.Text);
            Bipedal5Link.Bipedal5LinkHardware b = new Bipedal5Link.Bipedal5LinkHardware(m, m1, m2, l1, l2, new Bipedal5Link.PointD(0.92, 0.32), new double[] { theta, theta1, theta2, theta3, theta4 }, ss, Hardware);

            NewCentralPatternGenerator();

            timerUnstable.Stop();
            panelUnstable.Hide();

            int steps = int.Parse(textBoxSteps.Text);
            double deltatime = double.Parse(textBoxDeltaTime.Text);
            double u0 = double.Parse(textBoxCpgU0.Text);

            StatusBufferG.Clear(Color.Black);
            StatusBufferG.DrawString(B.Parameters(), F1, Brushes.DarkOrange, 0, 10);
            StatusBufferG.DrawString(CPG.Parameters(), F1, Brushes.DarkOrange, 440, 10);

            double[] joints;
            double[] feedback = new double[9];

            for (int s = 0 ; s < 100000 ; s++)
                joints = CPG.RunStep(u0, feedback, deltatime);

            Abort = false;
            labelHardwareStatus.Text = "starting..";
            System.Windows.Forms.Application.DoEvents();
            Hardware.Start();
            labelHardwareStatus.Text = "running..";

            // time coupling
            double simulationtime = 0;
            DateTime realtimestart = DateTime.Now;
            TimeSpan realtime;

            for (int s = 0 ; (s < steps) & !Abort ; s++)
            {
                // Run CPG step
                joints = CPG.RunStep(u0, new double[] { 0, feedback[1], -feedback[1], feedback[2], -feedback[2], 0, 0, 0, 0 }, deltatime);

                // Run Musculo-Skeletal system step
                feedback = b.RunStep(joints, deltatime);

                simulationtime = s * deltatime;
                realtime = DateTime.Now.Subtract(realtimestart);
                if (simulationtime > realtime.TotalSeconds)
                {
                    b.UpdateHardware();

                    DisplayBufferG.Clear(C1);
                    b.Draw(DisplayBufferG, new PointF(10, 240), 500);
                    DisplayBufferG.DrawString("running on hardware...", F1, Brushes.Orange, 550, 170);
                    LabelDisplayG.DrawImage(DisplayBuffer, 0, 0);

                    StatusBufferG.FillRectangle(Brushes.Black, 240, 0, 200, 160);
                    StatusBufferG.DrawString(b.Status(), F1, Brushes.DarkOrange, 240, 10);
                    StatusBufferG.FillRectangle(Brushes.Black, 640, 0, 200, 160);
                    StatusBufferG.DrawString(CPG.Status(), F1, Brushes.DarkOrange, 640, 10);
                    LabelStatusG.DrawImage(StatusBuffer, 0, 0);

                    CPGBufferG.Clear(Color.Black);
                    CPG.Draw(CPGBufferG);
                    LabelCPGG.DrawImage(CPGBuffer, 0, 0);

                    System.Windows.Forms.Application.DoEvents();

                }
            }

            labelHardwareStatus.Text = "stopping..";
            System.Windows.Forms.Application.DoEvents();
            Hardware.Stop();
            labelHardwareStatus.Text = "stopped";
        }

        private void linkLabelStopHardware_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Abort = true;
        }

        private void linkLabelResetHardware_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
            Hardware.Reset();
        }

        private void textBoxSteps_TextChanged(object sender, EventArgs e)
        {
            try
            {
                double st = double.Parse(textBoxSteps.Text);
                double dtt = double.Parse(textBoxDeltaTime.Text);
                double tt = st * dtt;
                labelTotalTime.Text = "total: " + tt.ToString("f2") + " sec";
            }
            catch
            {
            }
        }

        private void textBoxDeltaTime_TextChanged(object sender, EventArgs e)
        {
            try
            {
                double st = double.Parse(textBoxSteps.Text);
                double dtt = double.Parse(textBoxDeltaTime.Text);
                double tt = st * dtt;
                labelTotalTime.Text = "total: " + tt.ToString("f2") + " sec";
            }
            catch
            {
            }

        }

        private void textBoxSuper_TextChanged(object sender, EventArgs e)
        {
            labelSuper.Text = textBoxSuper.Text;
        }

    }


}