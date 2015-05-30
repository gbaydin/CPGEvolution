using System;
using System.Drawing;
using System.Media;
using System.IO.Ports;
using System.Threading;
using System.Collections;

namespace Bipedal5Link
{
    public struct PointD
    {
        public double X;
        public double Y;

        public PointD(double x, double y)
        {
            X = x;
            Y = y;
        }

        public static PointD operator +(PointD p1, PointD p2)
        {
            return new PointD(p1.X + p2.X, p1.Y + p2.Y);
        }

        public static PointD operator -(PointD p1, PointD p2)
        {
            return new PointD(p1.X - p2.X, p1.Y - p2.Y);
        }

        public static PointD operator *(PointD p1, double s)
        {
            return new PointD(p1.X * s, p1.Y * s);
        }

        public static PointD operator /(PointD p1, double s)
        {
            return new PointD(p1.X / s, p1.Y / s);
        }

        public static PointD Middle(PointD p1, PointD p2)
        {
            return new PointD((p1.X + p2.X) / 2, (p1.Y + p2.Y) / 2);
        }

        public static PointD FromPoint(PointD p1, double angle, double length)
        {
            return new PointD(p1.X + Math.Cos(angle) * length, p1.Y + Math.Sin(angle) * length);
        }

        public double Distance(PointD p)
        {
            return Math.Sqrt(Math.Pow(this.X - p.X, 2) + Math.Pow(this.Y - p.Y, 2));
        }
    }

    public class Bipedal5Link
    {
        public delegate double YGroundDelegate(double x);

        private double[] Theta;
        private PointD[] P;
        private bool ROnGround;
        private bool LOnGround;
        private PointD RZero;
        private PointD LZero;
        private double[] F;
        private double Torque;
        private double I;
        private PointD CenterOfMass;
        private PointD Velocity;
        private PointD Acceleration;
        private double AngularVelocity;
        private double AngularAcceleration;
        private double[] Joints;

        private double M;
        private double M1;
        private double M2;
        private double G;
        private double KGX;
        private double KGY;
        private double BGX;
        private double BGY;
        private double L1;
        private double L2;
        private double ServoSpeed;
        private YGroundDelegate YGround;
        private bool HipFix;

        private double M1PlusM2;
        private double M1PlusM2TimesTwo;
        private double MTotal;
        private double PiOverTwo;
        private double PiOverFour;
        private double ThreePiOverTwo;
        private double StableUpperLimit;
        private double[] Feedback;

        private Pen PenLeft;
        private Brush BrushLeft;
        private Pen PenRight;
        private Brush BrushRight;
        private Pen PenGround;
        private ArrayList LSteps;
        private ArrayList RSteps;
        private SoundPlayer S;

        public Bipedal5Link(double m, double m1, double m2, double l1, double l2, double kg, double bg, double g, PointD hip, double[] theta, double servoSpeed, YGroundDelegate yGround)
        {

            M = m;
            M1 = m1;
            M2 = m2;
            G = g;
            KGX = kg;
            KGY = KGX * 10;
            BGX = bg;
            BGY = BGX * 10;
            YGround = yGround;
            L1 = l1;
            L2 = l2;

            F = new double[5];
            Torque = 0;
            I = 1;
            Velocity = new PointD(0, 0);
            Acceleration = new PointD(0, 0);
            AngularVelocity = 0;
            AngularAcceleration = 0;
            Joints = new double[5];
            ServoSpeed = servoSpeed;
            Theta = new double[theta.Length];
            Array.Copy(theta, Theta, theta.Length);
            RZero = new PointD(0, 0);
            LZero = new PointD(0, 0);
            HipFix = false;
            Feedback = new double[9];

            P = new PointD[5];
            P[0] = hip;
            P[1] = PointD.FromPoint(P[0], Theta[0] - Theta[1], L1);
            P[2] = PointD.FromPoint(P[0], Theta[0] - Theta[2], L1);
            P[3] = PointD.FromPoint(P[1], Theta[0] - Theta[1] - Theta[3], L2);
            P[4] = PointD.FromPoint(P[2], Theta[0] - Theta[2] - Theta[4], L2);

            M1PlusM2 = M1 + M2;
            M1PlusM2TimesTwo = M1PlusM2 * 2;
            MTotal = M + 2 * M1 + 2 * M2;
            PiOverTwo = Math.PI / 2;
            PiOverFour = Math.PI / 4;
            ThreePiOverTwo = 3 * Math.PI / 2;
            StableUpperLimit = 1.3 * (L1 + L2);

            UpdateCenterOfMass();

            PenLeft = new Pen(Brushes.DarkOrange, 3);
            BrushLeft = new SolidBrush(Color.DarkOrange);
            PenRight = new Pen(Brushes.Yellow, 3);
            BrushRight = new SolidBrush(Color.Yellow);
            PenGround = new Pen(Brushes.SaddleBrown, 2);
            LSteps = new ArrayList(100);
            RSteps = new ArrayList(100);

            System.Reflection.Assembly a = System.Reflection.Assembly.GetExecutingAssembly();
            S = new SoundPlayer(a.GetManifestResourceStream("Bipedal5Link.step.wav"));

        }

        public double[] RunStep(double[] joints, double deltaTime)
        {
            Joints = joints;
            double j;
            
            j = Math.Min(Math.Max(joints[1], -1), 1) * ServoSpeed;
            Theta[1] += j * deltaTime;
            Theta[1] = Math.Min(Math.Max(Theta[1], PiOverTwo), ThreePiOverTwo);

            j = Math.Min(Math.Max(joints[2], -1), 1) * ServoSpeed;
            Theta[2] += j * deltaTime;
            Theta[2] = Math.Min(Math.Max(Theta[2], PiOverTwo), ThreePiOverTwo);

            j = Math.Min(Math.Max(joints[3], -1), 1) * ServoSpeed;
            Theta[3] += j * deltaTime;
            Theta[3] = Math.Min(Math.Max(Theta[3], 0), PiOverTwo);

            j = Math.Min(Math.Max(joints[4], -1), 1) * ServoSpeed;
            Theta[4] += j * deltaTime;
            Theta[4] = Math.Min(Math.Max(Theta[4], 0), PiOverTwo);

            //
            // Position propagation (top - down)
            //
            j = Theta[0] - Theta[1];
            P[1] = PointD.FromPoint(P[0], j, L1);
            P[3] = PointD.FromPoint(P[1], j - Theta[3], L2);
            j = Theta[0] - Theta[2];
            P[2] = PointD.FromPoint(P[0], j, L1);
            P[4] = PointD.FromPoint(P[2], j - Theta[4], L2);

            if (ROnGround)
            {
                if (P[3].Y > YGround(P[3].X))
                    ROnGround = false;
            }
            else
            {
                if (P[3].Y <= YGround(P[3].X))
                {
                    RZero = P[3];
                    ROnGround = true;
                    //if (sound)
                    //    S.Play();
                }
            }

            if (LOnGround)
            {
                if (P[4].Y > YGround(P[4].X))
                    LOnGround = false;
            }
            else
            {
                if (P[4].Y <= YGround(P[4].X))
                {
                    LZero = P[4];
                    LOnGround = true;
                    //if (sound)
                    //    S.Play();
                }
            }

            //
            // Force propagation (bottom - up)
            //
            if (ROnGround)
            {
                F[1] = -KGX * (P[3].X - RZero.X);
                F[2] = -KGY * (P[3].Y - RZero.Y);
            }
            else
            {
                F[1] = 0;
                F[2] = 0;
            }

            if (LOnGround)
            {
                F[3] = -KGX * (P[4].X - LZero.X);
                F[4] = -KGY * (P[4].Y - LZero.Y);
            }
            else
            {
                F[3] = 0;
                F[4] = 0;
            }

            //
            // Update accelerations & velocities & positions
            //
            UpdateCenterOfMass();

            if (LOnGround || ROnGround)
            {
                if (ROnGround)
                {
                    I = MTotal * Math.Pow(CenterOfMass.Distance(RZero), 2);
                }
                else
                {
                    I = MTotal * Math.Pow(CenterOfMass.Distance(LZero), 2);
                }
                Torque = (P[3].X - CenterOfMass.X) * F[2] + (CenterOfMass.Y - P[3].Y) * F[1] + (P[4].X - CenterOfMass.X) * F[4] + (CenterOfMass.Y - P[4].Y) * F[3];
                AngularAcceleration = Torque / I;
                AngularVelocity += AngularAcceleration * deltaTime;
            }
            Theta[0] += AngularVelocity * deltaTime;

            Acceleration.X = (F[1] + F[3]) / MTotal;
            Acceleration.Y = (F[2] + F[4]) / MTotal - G;
            if (!HipFix)
                P[0] += Velocity * deltaTime;

            Velocity += Acceleration * deltaTime;

            //Stabilize
            Velocity *= 0.9999;
            AngularVelocity *= 0.9999;

            //Hardware
            //UpdateHardware();

            //Feedback
            double theta1v = PiOverTwo + (Theta[0] - Theta[1]);
            double theta2v = PiOverTwo + (Theta[0] - Theta[2]);
            double theta3v = PiOverTwo + (Theta[0] - Theta[1] - Theta[3]);
            double theta4v = PiOverTwo + (Theta[0] - Theta[2] - Theta[4]);
            Feedback[1] = theta1v;
            Feedback[2] = theta2v;
            Feedback[3] = theta3v;
            Feedback[4] = theta4v;
            if (ROnGround)
                Feedback[5] = 1;
            else
                Feedback[5] = 0;
            if (LOnGround)
                Feedback[6] = 1;
            else
                Feedback[6] = 0;
            return Feedback;
        }

        private void UpdateCenterOfMass()
        {
            PointD comr = (PointD.Middle(P[0], P[1]) * M1 + PointD.Middle(P[1], P[3]) * M2) / M1PlusM2;
            PointD coml = (PointD.Middle(P[0], P[2]) * M1 + PointD.Middle(P[2], P[4]) * M2) / M1PlusM2;
            CenterOfMass = (PointD.Middle(coml, comr) * M1PlusM2TimesTwo + P[0] * M) / MTotal;
        }

        public bool IsStable()
        {
            double ground = YGround(P[0].X);
            //return (P[0].Y > P[1].Y) && (P[0].Y > P[2].Y) && (P[0].Y > ground) && (P[0].Y < ground + StableUpperLimit);
            //return (P[0].Y > P[1].Y) && (P[0].Y > P[2].Y) && (P[0].Y > ground) && (P[0].Y < ground + StableUpperLimit);
            //return (P[0].Y > ground) && (P[1].Y > ground) && (P[2].Y > ground) && (P[0].Y < ground + StableUpperLimit);
            return (P[0].Y > ground) && (P[1].Y > ground) && (P[2].Y > ground);
            //return (P[0].Y > P[1].Y) && (P[0].Y > P[2].Y);
            //return (P[0].Y > ground);
        }

        public PointD BehindPosition()
        {
            int b = 0;
            for (int i = 1 ; i < 5 ; i++)
                if (P[i].X < P[b].X)
                    b = i;
            return P[b];
        }

        public PointD ForemostPosition()
        {
            int b = 0;
            for (int i = 1 ; i < 5 ; i++)
                if (P[i].X > P[b].X)
                    b = i;
            return P[b];
        }

        public double Theta1
        {
            get
            {
                return Theta[1];
            }
        }

        public double Theta2
        {
            get
            {
                return Theta[2];
            }
        }

        public PointD HipPosition
        {
            get
            {
                return P[0];
            }
        }

        public bool HipFixed
        {
            get
            {
                return HipFix;
            }
            set
            {
                HipFix = value;
            }
        }

        public bool LeftFootOnTheGround
        {
            get
            {
                return LOnGround;
            }
        }

        public bool RightFootOnTheGround
        {
            get
            {
                return ROnGround;
            }
        }

        public void Draw(Graphics g, PointF p, float scale)
        {
            PointF g1 = new PointF(p.X, p.Y - (float)YGround(0) * scale);
            PointF g2;
            for (float x = 100 ; x < 1500 ; x += 100)
            {
                g2 = new PointF(p.X + x * scale, p.Y - (float)YGround(x) * scale);
                g.DrawLine(PenGround, g1, g2);
                g1 = g2;
            }

            PointF hip = new PointF(p.X + (float)P[0].X * scale, p.Y - (float)P[0].Y * scale);
            PointF kneel = new PointF(p.X + (float)P[2].X * scale, p.Y - (float)P[2].Y * scale);
            PointF kneer = new PointF(p.X + (float)P[1].X * scale, p.Y - (float)P[1].Y * scale);
            PointF anklel = new PointF(p.X + (float)P[4].X * scale, p.Y - (float)P[4].Y * scale);
            PointF ankler = new PointF(p.X + (float)P[3].X * scale, p.Y - (float)P[3].Y * scale);
            PointF centerofmass = new PointF(p.X + (float)CenterOfMass.X * scale, p.Y - (float)CenterOfMass.Y * scale);

            PointD h = PointD.FromPoint(P[0], Theta[0], 0.07);
            PointF head = new PointF(p.X + (float)h.X * scale, p.Y - (float)h.Y * scale);

            g.FillEllipse(Brushes.Blue, centerofmass.X - 3, centerofmass.Y - 3, 6, 6);

            g.DrawLine(PenGround, hip, head);
            
            g.DrawLine(PenLeft, hip, kneel);
            g.DrawLine(PenLeft, kneel, anklel);
            g.FillEllipse(BrushLeft, kneel.X - 5, kneel.Y - 5, 10, 10);

            g.FillEllipse(BrushRight, hip.X - 5, hip.Y - 5, 10, 10);

            g.DrawLine(PenRight, hip, kneer);
            g.DrawLine(PenRight, kneer, ankler);
            g.FillEllipse(BrushRight, kneer.X - 5, kneer.Y - 5, 10, 10);

            if (HipFix)
                g.DrawEllipse(PenRight, hip.X - 8, hip.Y - 8, 16, 16);

            if (LOnGround)
                LSteps.Add(new RectangleF(anklel.X, p.Y + 3, 1, 1));
            if (ROnGround)
                RSteps.Add(new RectangleF(ankler.X, p.Y + 6, 1, 1));

            RectangleF[] ls = new RectangleF[LSteps.Count];
            LSteps.CopyTo(ls);
            if (ls.Length > 0)
                g.FillRectangles(BrushLeft, ls);
            RectangleF[] rs = new RectangleF[RSteps.Count];
            RSteps.CopyTo(rs);
            if (rs.Length > 0)
                g.FillRectangles(BrushRight, rs);
        }


        public string Parameters()
        {
            string ret = "musculo - skeletal system parameters" + Environment.NewLine;
            ret += "M = " + M.ToString("f3") + " kg" + Environment.NewLine;
            ret += "m1 = " + M1.ToString("f3") + " kg" + Environment.NewLine;
            ret += "l1 = " + L1.ToString("f3") + " m" + Environment.NewLine;
            ret += "m2 = " + M2.ToString("f3") + " kg" + Environment.NewLine;
            ret += "l2 = " + L2.ToString("f3") + " m" + Environment.NewLine;
            ret += "kgx = " + KGX.ToString("f3") + Environment.NewLine;
            ret += "kgy = " + KGY.ToString("f3") + Environment.NewLine;
            ret += "g = " + G.ToString("f3") + " kg m / s2" + Environment.NewLine;
            return ret;
        }

        public string Status()
        {
            string ret = "musculo - skeletal system status" + Environment.NewLine;
            for (int i = 0 ; i < 5 ; i++)
                ret += "(P" + i.ToString() + ".X, P" + i.ToString() + ".Y) = (" + P[i].X.ToString("f3") + " m, " + P[i].Y.ToString("f3") + " m)" + Environment.NewLine;
            ret += "Torque = " + Torque.ToString("f3") + " Nm" + Environment.NewLine;
            ret += "I = " + I.ToString("f3") + " kgm2" + Environment.NewLine;
            ret += "Velocity = (" + Velocity.X.ToString("f3") + " m/s, " + Velocity.Y.ToString("f3") + " m/s)" + Environment.NewLine;
            ret += "AngularVelocity = " + AngularVelocity.ToString("f3") + " radian/s" + Environment.NewLine;
            if (LOnGround)
                ret += "left foot on the ground" + Environment.NewLine;
            else
                ret += "left foot above the ground" + Environment.NewLine;
            if (ROnGround)
                ret += "right foot on the ground" + Environment.NewLine;
            else
                ret += "right foot above the ground" + Environment.NewLine;
            return ret;
        }
    }

    public class Bipedal5LinkHardware
    {
        private double[] Theta;
        private PointD[] P;
        private bool ROnGround;
        private bool LOnGround;
        private double[] Joints;

        private double M;
        private double M1;
        private double M2;
        private double L1;
        private double L2;
        private double ServoSpeed;
        private bool HipFix;

        private double M1PlusM2;
        private double M1PlusM2TimesTwo;
        private double MTotal;
        private PointD CenterOfMass;
        private double PiOverTwo;
        private double PiOverFour;
        private double ThreePiOverTwo;
        private double[] Feedback;

        private Pen PenLeft;
        private Brush BrushLeft;
        private Pen PenRight;
        private Brush BrushRight;
        private Pen PenGround;
        private ArrayList LSteps;
        private ArrayList RSteps;
        private SoundPlayer S;
        private ServoController Hardware;

        public Bipedal5LinkHardware(double m, double m1, double m2, double l1, double l2, PointD hip, double[] theta, double servoSpeed, ServoController hardware)
        {

            M = m;
            M1 = m1;
            M2 = m2;
            L1 = l1;
            L2 = l2;

            Joints = new double[5];
            ServoSpeed = servoSpeed;
            Theta = new double[theta.Length];
            Array.Copy(theta, Theta, theta.Length);
            Feedback = new double[9];

            P = new PointD[5];
            P[0] = hip;
            P[1] = PointD.FromPoint(P[0], Theta[0] - Theta[1], L1);
            P[2] = PointD.FromPoint(P[0], Theta[0] - Theta[2], L1);
            P[3] = PointD.FromPoint(P[1], Theta[0] - Theta[1] - Theta[3], L2);
            P[4] = PointD.FromPoint(P[2], Theta[0] - Theta[2] - Theta[4], L2);

            M1PlusM2 = M1 + M2;
            M1PlusM2TimesTwo = M1PlusM2 * 2;
            MTotal = M + 2 * M1 + 2 * M2;
            PiOverTwo = Math.PI / 2;
            PiOverFour = Math.PI / 4;
            ThreePiOverTwo = 3 * Math.PI / 2;

            UpdateCenterOfMass();

            PenLeft = new Pen(Brushes.DarkOrange, 3);
            BrushLeft = new SolidBrush(Color.DarkOrange);
            PenRight = new Pen(Brushes.Yellow, 3);
            BrushRight = new SolidBrush(Color.Yellow);
            PenGround = new Pen(Brushes.SaddleBrown, 2);
            LSteps = new ArrayList(100);
            RSteps = new ArrayList(100);

            System.Reflection.Assembly a = System.Reflection.Assembly.GetExecutingAssembly();
            S = new SoundPlayer(a.GetManifestResourceStream("Bipedal5Link.step.wav"));

            Hardware = hardware;
        }

        public double[] RunStep(double[] joints, double deltaTime)
        {
            Joints = joints;
            double j;

            j = Math.Min(Math.Max(joints[1], -1), 1) * ServoSpeed;
            Theta[1] += j * deltaTime;
            Theta[1] = Math.Min(Math.Max(Theta[1], PiOverTwo), ThreePiOverTwo);

            j = Math.Min(Math.Max(joints[2], -1), 1) * ServoSpeed;
            Theta[2] += j * deltaTime;
            Theta[2] = Math.Min(Math.Max(Theta[2], PiOverTwo), ThreePiOverTwo);

            j = Math.Min(Math.Max(joints[3], -1), 1) * ServoSpeed;
            Theta[3] += j * deltaTime;
            Theta[3] = Math.Min(Math.Max(Theta[3], 0), PiOverTwo);

            j = Math.Min(Math.Max(joints[4], -1), 1) * ServoSpeed;
            Theta[4] += j * deltaTime;
            Theta[4] = Math.Min(Math.Max(Theta[4], 0), PiOverTwo);

            //
            // Position propagation (top - down)
            //
            j = Theta[0] - Theta[1];
            P[1] = PointD.FromPoint(P[0], j, L1);
            P[3] = PointD.FromPoint(P[1], j - Theta[3], L2);
            j = Theta[0] - Theta[2];
            P[2] = PointD.FromPoint(P[0], j, L1);
            P[4] = PointD.FromPoint(P[2], j - Theta[4], L2);

            UpdateCenterOfMass();

            //Feedback

            Feedback[1] = Theta[1] - Math.PI;
            Feedback[2] = Theta[2] - Math.PI;
            //Feedback[3] = Theta[3] - PiOverFour;
            //Feedback[4] = Theta[4] - PiOverFour;
            return Feedback;
        }

        private void UpdateCenterOfMass()
        {
            PointD comr = (PointD.Middle(P[0], P[1]) * M1 + PointD.Middle(P[1], P[3]) * M2) / M1PlusM2;
            PointD coml = (PointD.Middle(P[0], P[2]) * M1 + PointD.Middle(P[2], P[4]) * M2) / M1PlusM2;
            CenterOfMass = (PointD.Middle(coml, comr) * M1PlusM2TimesTwo + P[0] * M) / MTotal;
        }

        public double Theta1
        {
            get
            {
                return Theta[1];
            }
        }

        public double Theta2
        {
            get
            {
                return Theta[2];
            }
        }

        public PointD HipPosition
        {
            get
            {
                return P[0];
            }
        }

        public bool LeftFootOnTheGround
        {
            get
            {
                return LOnGround;
            }
        }

        public bool RightFootOnTheGround
        {
            get
            {
                return ROnGround;
            }
        }

        public void UpdateHardware()
        {
            if (Hardware != null)
            {
                byte j0 = (byte)Math.Min(((Theta[1] - PiOverTwo) / Math.PI) * 225, 225);
                byte j1 = (byte)Math.Min(225 - ((Theta[2] - PiOverTwo) / Math.PI) * 225, 225);
                byte j2 = (byte)Math.Min(112 - (Theta[3] / Math.PI) * 225, 225);
                byte j3 = (byte)Math.Min((Theta[4] / Math.PI) * 225, 225);
                Hardware.SetPositions(new byte[] { j0, j1, j2, j3 });
                //Hardware.SetPosition(0, (byte)(((Theta[1] - PiOverTwo) / Math.PI) * 225));
                //Hardware.SetPosition(1, (byte)(225 - ((Theta[2] - PiOverTwo) / Math.PI) * 225));
                //Hardware.SetPosition(2, (byte)(112 - (Theta[3] / Math.PI) * 225));
                //Hardware.SetPosition(3, (byte)((Theta[4] / Math.PI) * 225));
            }
        }

        public void Draw(Graphics g, PointF p, float scale)
        {
            PointF hip = new PointF(p.X + (float)P[0].X * scale, p.Y - (float)P[0].Y * scale);
            PointF kneel = new PointF(p.X + (float)P[2].X * scale, p.Y - (float)P[2].Y * scale);
            PointF kneer = new PointF(p.X + (float)P[1].X * scale, p.Y - (float)P[1].Y * scale);
            PointF anklel = new PointF(p.X + (float)P[4].X * scale, p.Y - (float)P[4].Y * scale);
            PointF ankler = new PointF(p.X + (float)P[3].X * scale, p.Y - (float)P[3].Y * scale);
            PointF centerofmass = new PointF(p.X + (float)CenterOfMass.X * scale, p.Y - (float)CenterOfMass.Y * scale);

            PointD h = PointD.FromPoint(P[0], Theta[0], 0.07);
            PointF head = new PointF(p.X + (float)h.X * scale, p.Y - (float)h.Y * scale);

            g.FillEllipse(Brushes.Blue, centerofmass.X - 3, centerofmass.Y - 3, 6, 6);

            g.DrawLine(PenGround, hip, head);

            g.DrawLine(PenLeft, hip, kneel);
            g.DrawLine(PenLeft, kneel, anklel);
            g.FillEllipse(BrushLeft, kneel.X - 5, kneel.Y - 5, 10, 10);

            g.FillEllipse(BrushRight, hip.X - 5, hip.Y - 5, 10, 10);

            g.DrawLine(PenRight, hip, kneer);
            g.DrawLine(PenRight, kneer, ankler);
            g.FillEllipse(BrushRight, kneer.X - 5, kneer.Y - 5, 10, 10);

            if (HipFix)
                g.DrawEllipse(PenRight, hip.X - 8, hip.Y - 8, 16, 16);

            if (LOnGround)
                LSteps.Add(new RectangleF(anklel.X, p.Y + 3, 1, 1));
            if (ROnGround)
                RSteps.Add(new RectangleF(ankler.X, p.Y + 6, 1, 1));

            RectangleF[] ls = new RectangleF[LSteps.Count];
            LSteps.CopyTo(ls);
            if (ls.Length > 0)
                g.FillRectangles(BrushLeft, ls);
            RectangleF[] rs = new RectangleF[RSteps.Count];
            RSteps.CopyTo(rs);
            if (rs.Length > 0)
                g.FillRectangles(BrushRight, rs);
        }

        public string Parameters()
        {
            string ret = "musculo - skeletal system parameters" + Environment.NewLine;
            ret += "M = " + M.ToString("f3") + " kg" + Environment.NewLine;
            ret += "m1 = " + M1.ToString("f3") + " kg" + Environment.NewLine;
            ret += "l1 = " + L1.ToString("f3") + " m" + Environment.NewLine;
            ret += "m2 = " + M2.ToString("f3") + " kg" + Environment.NewLine;
            ret += "l2 = " + L2.ToString("f3") + " m" + Environment.NewLine;
            return ret;
        }

        public string Status()
        {
            string ret = "musculo - skeletal system status" + Environment.NewLine;
            for (int i = 0 ; i < 5 ; i++)
                ret += "(P" + i.ToString() + ".X, P" + i.ToString() + ".Y) = (" + P[i].X.ToString("f3") + " m, " + P[i].Y.ToString("f3") + " m)" + Environment.NewLine;
            if (LOnGround)
                ret += "left foot on the ground" + Environment.NewLine;
            else
                ret += "left foot above the ground" + Environment.NewLine;
            if (ROnGround)
                ret += "right foot on the ground" + Environment.NewLine;
            else
                ret += "right foot above the ground" + Environment.NewLine;
            return ret;
        }
    }

    public class CPGNeuron
    {
        public double TauU;
        public double TauV;
        public double Beta;

        public double U;
        public double V;
        public double Y;

        public CPGNeuron(double tauU, double tauV, double beta)
        {
            TauU = tauU;
            TauV = tauV;
            Beta = beta;

            Random rnd = new Random();
            U = 0;
            V = 0;
            Y = 0;
        }

        public double RunStep(double activation, double u0, double feedback, double deltaTime)
        {
            double d = -U - Beta * V + activation + u0 + feedback;
            U += deltaTime * (d / TauU);
            V += deltaTime * (-V + Y) / TauV;
            Y = Math.Max(0, U);
            return Y;
        }

    }

    public class CentralPatternGenerator
    {
        private CPGNeuron[] Neuron;
        private double[,] Weights;
        private PointF[] DrawingCoordinates;
        private Font F1;

        public CentralPatternGenerator(double hipTauU, double hipTauV, double hipBeta, double kneeTauU, double kneeTauV, double kneeBeta, double wFlexorExtensor, double wHipKnee, double wLeftRight)
        {
            Neuron = new CPGNeuron[9];
            Weights = new double[9, 9];

            Weights[1, 2] = wFlexorExtensor;
            Weights[2, 1] = wFlexorExtensor;
            Weights[3, 4] = wFlexorExtensor;
            Weights[4, 3] = wFlexorExtensor;
            Weights[5, 6] = wFlexorExtensor;
            Weights[6, 5] = wFlexorExtensor;
            Weights[7, 8] = wFlexorExtensor;
            Weights[8, 7] = wFlexorExtensor;

            Weights[1, 3] = wLeftRight;
            Weights[3, 1] = wLeftRight;
            Weights[2, 4] = wLeftRight;
            Weights[4, 2] = wLeftRight;

            Weights[5, 2] = wHipKnee;
            Weights[6, 1] = wHipKnee;
            Weights[7, 4] = wHipKnee;
            Weights[8, 3] = wHipKnee;

            Neuron[1] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[1].U = 0.1;
            Neuron[2] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[5] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);
            Neuron[6] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);

            Neuron[3] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[4] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[7] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);
            Neuron[8] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);

            DrawingCoordinates = new PointF[9];
            DrawingCoordinates[1] = new PointF(175, 30);
            DrawingCoordinates[2] = new PointF(200, 70);
            DrawingCoordinates[5] = new PointF(175, 160);
            DrawingCoordinates[6] = new PointF(200, 200);
            DrawingCoordinates[3] = new PointF(50, 30);
            DrawingCoordinates[4] = new PointF(25, 70);
            DrawingCoordinates[7] = new PointF(50, 160);
            DrawingCoordinates[8] = new PointF(25, 200);
            F1 = new Font("Arial", 8);
        }

        public CentralPatternGenerator(double hipTauU, double hipTauV, double hipBeta, double kneeTauU, double kneeTauV, double kneeBeta, double[,] weights)
        {
            Neuron = new CPGNeuron[9];
            Weights = weights;

            Neuron[1] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[1].U = 0.1;
            Neuron[2] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[5] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);
            Neuron[6] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);

            Neuron[3] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[4] = new CPGNeuron(hipTauU, hipTauV, hipBeta);
            Neuron[7] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);
            Neuron[8] = new CPGNeuron(kneeTauU, kneeTauV, kneeBeta);

            DrawingCoordinates = new PointF[9];
            DrawingCoordinates[1] = new PointF(175, 30);
            DrawingCoordinates[2] = new PointF(200, 70);
            DrawingCoordinates[5] = new PointF(175, 160);
            DrawingCoordinates[6] = new PointF(200, 200);
            DrawingCoordinates[3] = new PointF(50, 30);
            DrawingCoordinates[4] = new PointF(25, 70);
            DrawingCoordinates[7] = new PointF(50, 160);
            DrawingCoordinates[8] = new PointF(25, 200);
            F1 = new Font("Arial", 8);
        }

        public string Parameters()
        {
            string ret = "central pattern generator parameters" + Environment.NewLine;
            ret += "hTauU = " + Neuron[1].TauU.ToString("f3") + Environment.NewLine;
            ret += "hTauV = " + Neuron[1].TauV.ToString("f3") + Environment.NewLine;
            ret += "hBeta = " + Neuron[1].Beta.ToString("f3") + Environment.NewLine;
            ret += "kTauU = " + Neuron[5].TauU.ToString("f3") + Environment.NewLine;
            ret += "kTauV = " + Neuron[5].TauV.ToString("f3") + Environment.NewLine;
            ret += "kBeta = " + Neuron[5].Beta.ToString("f3") + Environment.NewLine;

            return ret;
        }

        public string Status()
        {
            string ret = "central pattern generator status" + Environment.NewLine;
            ret += "right hip  flexor y = " + Neuron[1].Y.ToString("f3") + Environment.NewLine;
            ret += "right hip  extensor y = " + Neuron[2].Y.ToString("f3") + Environment.NewLine;
            ret += "left  hip  flexor y = " + Neuron[3].Y.ToString("f3") + Environment.NewLine;
            ret += "left  hip  extensor y = " + Neuron[4].Y.ToString("f3") + Environment.NewLine;
            ret += "right knee flexor y = " + Neuron[5].Y.ToString("f3") + Environment.NewLine;
            ret += "right knee extensor y = " + Neuron[6].Y.ToString("f3") + Environment.NewLine;
            ret += "left  knee flexor y = " + Neuron[7].Y.ToString("f3") + Environment.NewLine;
            ret += "left  knee extensor y = " + Neuron[8].Y.ToString("f3") + Environment.NewLine;
            return ret;
        }

        public double[] RunStep(double u0, double[] feedback, double deltaTime)
        {
            double[] y = new double[9];
            for (int i = 1 ; i < 9 ; i++)
                y[i] = Neuron[i].Y;

            double activation;
            for (int i = 1 ; i < 9 ; i++)
            {
                activation = 0;
                for (int j = 1 ; j < 9 ; j++)
                    activation += Weights[i, j] * y[j];
                Neuron[i].RunStep(activation, u0, feedback[i], deltaTime);
            }
            
            double[] ret = new double[5];
            ret[1] = - Neuron[1].Y + Neuron[2].Y;
            ret[2] = - Neuron[3].Y + Neuron[4].Y;

            ret[3] = Neuron[5].Y - Neuron[6].Y;
            ret[4] = Neuron[7].Y - Neuron[8].Y;

            return ret;
        }

        public void Draw(Graphics g)
        {
            for (int i = 1 ; i < 9 ; i++)
                for (int j = 1 ; j < 9 ; j++)
                {
                    if (Weights[i, j] != 0)
                        g.DrawLine(Pens.SaddleBrown, DrawingCoordinates[j], DrawingCoordinates[i]);
                }

            double u;
            int c;
            Brush b = Brushes.Blue;
            for (int i = 1 ; i < 9 ; i++)
            {
                u = Math.Max(Math.Min(Neuron[i].U, 0.8), -0.8);
                if ((-0.8 <= u) && (u <= 0.8))
                {
                    c = (int)(318.0 * u);
                    if (c > 0)
                        b = new SolidBrush(Color.FromArgb(c, c, 0));
                    else
                        b = new SolidBrush(Color.FromArgb(-c, 0, 0));
                }

                g.FillEllipse(b, DrawingCoordinates[i].X - 18, DrawingCoordinates[i].Y - 18, 36, 36);
                g.DrawEllipse(Pens.SaddleBrown, DrawingCoordinates[i].X - 18, DrawingCoordinates[i].Y - 18, 36, 36);
                g.DrawString("U" + i.ToString(), F1, Brushes.DarkOrange, DrawingCoordinates[i].X + 25, DrawingCoordinates[i].Y + 2);
            }

        }
    }

    public class ServoController
    {
        private SerialPort Port;
        public bool Running;
        private byte[] Buffer;
        private byte Response;
        private const int CommandStart = 230;
        private const int CommandReset = 231;
        private const int CommandStop = 232;
        private const int ResponseOK = 253;

        public ServoController(string comPort)
        {
            Buffer = new byte[4];
            Running = false;
            Port = new SerialPort(comPort, 9600, Parity.None, 8, StopBits.One);
            Port.DataReceived += new SerialDataReceivedEventHandler(Port_DataReceived);
            //Port.ReadTimeout = 20;
        }

        ~ServoController()
        {
            if (Port != null)
                if (Port.IsOpen)
                    Port.Close();
        }

        public void Start()
        {
            if (!Running)
            {
                try
                {
                    Port.Open();
                }
                catch
                {
                }
                if (Port.IsOpen)
                {
                    //Buffer[0] = CommandStart;
                    //Port.Write(Buffer, 0, 4);
                    Running = true;
                }
            }
        }

        public void Stop()
        {
            //if (Running)
            //{
            Running = false;
            //for (int i = 0 ; i < 10 ; i++)
            //{
            //    Buffer[0] = CommandStop;
            //    Port.Write(Buffer, 0, 4);
            //    Thread.Sleep(10);
            //}
            //}
        }

        public void Reset()
        {
            if (!Running)
            {
                //Buffer[0] = CommandReset;
                //Port.Write(Buffer, 0, 4);

                byte j0p = Buffer[0];
                byte j1p = Buffer[1];
                byte j2p = Buffer[2];
                byte j3p = Buffer[3];
                for (int st = 0 ; st < 50 ; st++)
                {
                    Buffer[0] = (byte)(j0p + ((112 - j0p) * ((float)st + 1f) / 50));
                    Buffer[1] = (byte)(j1p + ((112 - j1p) * ((float)st + 1f) / 50));
                    Buffer[2] = (byte)(j2p + ((112 - j2p) * ((float)st + 1f) / 50));
                    Buffer[3] = (byte)(j3p + ((0 - j3p) * ((float)st + 1f) / 50));
                    Port.Write(Buffer, 0, 4);
                    Thread.Sleep(20);
                }
            }
        }

        public void SetPosition(int index, byte position)
        {
            if (Running)
            {
                Buffer[index] = position;
                Port.Write(Buffer, 0, 4);
            }
        }

        public void SetPositions(byte[] positions)
        {
            if (Running)
            {
                Buffer[0] = positions[0];
                Buffer[1] = positions[1];
                Buffer[2] = positions[2];
                Buffer[3] = positions[3];
                Port.Write(Buffer, 0, 4);
            }
        }

        public void Port_DataReceived(Object sender, SerialDataReceivedEventArgs e)
        {
            Response = (byte)Port.ReadByte();
        }
    }
}
