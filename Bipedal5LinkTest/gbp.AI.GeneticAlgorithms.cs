using System;
using System.Drawing;
using System.Threading;
using System.Collections;
using gbp.Drawing.Graph;

namespace gbp
{
    namespace AI
    {
        namespace GeneticAlgorithms
        {
            public class Population2
            {
                internal int Size;
                private int ChromosomeLength;
                private float NucleotideMin;
                private float NucleotideMax;
                private float NucleotideRange;
                internal float[][] Individuals;
                private Random rnd;
                internal float[] Fitnesses;
                internal int BestIndividualIndex;

                public Population2(int size, int chromosomeLength, float nucleotideMin, float nucleotideMax)
                {
                    rnd = new Random();
                    Size = size;
                    ChromosomeLength = chromosomeLength;
                    NucleotideMin = nucleotideMin;
                    NucleotideMax = nucleotideMax;
                    NucleotideRange = NucleotideMax - NucleotideMin;
                    Individuals = new float[Size][];
                    for (int i = 0 ; i < Size ; i++)
                    {
                        Individuals[i] = new float[ChromosomeLength];
                        for (int n = 0 ; n < ChromosomeLength ; n++)
                            Individuals[i][n] = (float)(NucleotideMin + rnd.NextDouble() * NucleotideRange);
                    }
                    BestIndividualIndex = 0;
                }

                public void NextGeneration(float crossoverChance, float mutationChance, int tournamentSize, float tournamentChance, bool elitism)
                {
                    float[][] ng = new float[Size][];
                    int crossovers = (int)(Size * crossoverChance / 2);
                    int reproductions = Size - crossovers * 2;
                    int pos = 0;
                    float[] r;
                    for (int i = 0 ; i < reproductions ; i++)
                    {
                        r = Individuals[AGoodChromosomeIndex(tournamentSize, tournamentChance)];
                        ng[pos] = new float[ChromosomeLength];
                        for (int j = 0 ; j < ChromosomeLength ; j++)
                            ng[pos][j] = r[j];
                        pos++;
                    }
                    float[][] c;
                    for (int i = 0 ; i < crossovers ; i++)
                    {
                        c = Crossover(Individuals[AGoodChromosomeIndex(tournamentSize, tournamentChance)], Individuals[AGoodChromosomeIndex(tournamentSize, tournamentChance)]);
                        ng[pos] = c[0];
                        ng[pos + 1] = c[1];
                        pos += 2;
                    }
                    int mutations = (int)(Size * mutationChance);
                    for (int i = 0 ; i < mutations ; i++)
                        Mutate(ng[rnd.Next(Size)]);

                    if (elitism)
                        ng[0] = Individuals[BestIndividualIndex];

                    Individuals = ng;
                }

                private float[][] CrossoverOld(float[] parent1, float[] parent2)
                {
                    float[][] children = new float[2][];
                    children[0] = new float[ChromosomeLength];
                    children[1] = new float[ChromosomeLength];
                    int cut1 = rnd.Next(ChromosomeLength);
                    int cut2 = rnd.Next(cut1, ChromosomeLength + 1);
                    for (int i = 0 ; i < cut1 ; i++)
                    {
                        children[0][i] = parent1[i];
                        children[1][i] = parent2[i];
                    }
                    for (int i = cut1 ; i < cut2 ; i++)
                    {
                        children[0][i] = parent2[i];
                        children[1][i] = parent1[i];
                    }
                    for (int i = cut2 ; i < ChromosomeLength ; i++)
                    {
                        children[0][i] = parent1[i];
                        children[1][i] = parent2[i];
                    }
                    return children;
                }

                private float[][] Crossover(float[] parent1, float[] parent2)
                {
                    float[][] children = new float[2][];
                    children[0] = new float[ChromosomeLength];
                    children[1] = new float[ChromosomeLength];
                    int cut1 = rnd.Next(ChromosomeLength);
                    int cut2 = rnd.Next(cut1, ChromosomeLength + 1);
                    for (int i = 0 ; i < cut1 ; i++)
                    {
                        children[0][i] = parent1[i];
                        children[1][i] = parent2[i];
                    }
                    for (int i = cut1 ; i < cut2 ; i++)
                    {
                        children[0][i] = (parent2[i] + parent1[i]) / 2f;
                        children[1][i] = children[0][i];
                    }
                    for (int i = cut2 ; i < ChromosomeLength ; i++)
                    {
                        children[0][i] = parent1[i];
                        children[1][i] = parent2[i];
                    }
                    return children;
                }
                
                private void MutateOld(float[] c)
                {
                    c[rnd.Next(ChromosomeLength)] = (float)(NucleotideMin + rnd.NextDouble() * NucleotideRange);
                }

                private void Mutate(float[] c)
                {
                    int gene = rnd.Next(ChromosomeLength);
                    c[gene] *= (float)(0.8 + 0.4 * rnd.NextDouble());
                    c[gene] = Math.Max(Math.Min(NucleotideMax, c[gene]), NucleotideMin);
                }

                private int AGoodChromosomeIndex(int tournamentSize, float tournamentChance)
                {
                    if (tournamentSize == -1)
                    {
                        double hit = rnd.NextDouble();
                        double pos = 0;
                        for (int i = 0 ; i < Size ; i++)
                        {
                            pos += Fitnesses[i];
                            if (pos >= hit)
                                return i;
                        }
                        return Size - 1;
                    }
                    else
                    {
                        int ret = rnd.Next(Size);
                        int participant;
                        for (int i = 0 ; i < tournamentSize - 1 ; i++)
                        {
                            participant = rnd.Next(Size);
                            if (Fitnesses[participant] >= Fitnesses[ret])
                                if (tournamentChance > rnd.NextDouble())
                                    ret = participant;
                        }
                        return ret;
                    }
                }

                public string Representation(float[] individual)
                {
                    string ret = individual[0].ToString();
                    for (int i = 1 ; i < individual.Length ; i++)
                        ret += ", " + individual[i].ToString();
                    return ret;
                }

                internal bool IsEqual(float[] individual1, float[] individual2)
                {
                    for (int i = 0 ; i < ChromosomeLength ; i++)
                        if (individual1[i] != individual2[i])
                            return false;
                    return true;
                }
            }

            public abstract class GeneticAlgorithms
            {
                private Individual[] Individuals;
                private int Size;
                private int GenomeLength;
                private double GeneMinimum;
                private double GeneMaximum;

                private int Generations;
                private float CrossoverChance;
                private float MutationChance;
                private int TournamentSize;
                private float TournamentChance;
                private bool Elitism;
                private float FitnessTreshold;
                private SolutionFoundDelegate SF;
                private StoppedDelegate STP;
                protected ImprovementDelegate IMP;
                private Graphics GReport;
                protected Graphics GReport2;
                protected Graphics GReport3;
                protected Graphics GFitness;
                private Bitmap GReportBuffer;
                protected Bitmap GReport2Buffer;
                protected Bitmap GReport3Buffer;
                protected Bitmap GFitnessBuffer;
                private Graphics GReportBufferG;
                protected Graphics GReport2BufferG;
                protected Graphics GReport3BufferG;
                protected Graphics GFitnessBufferG;
                private Color BackColor;
                private Brush ForeColor1;
                private Brush ForeColor2;
                private Thread T;
                private bool Abort;
                private ArrayList GenerationBests;
                private bool ShowEval;
                private bool ShowImprove;
                private Font f;
                private Font f2;
                private int BestIndex;
                private int WorstIndex;
                private Individual BestIndividual;
                private float AverageRawFitness;
                private float Diversity;
                protected bool IndividualsDrawn;
                private Random rnd;


                public GeneticAlgorithms(int size, int genomeLength, double geneMinimum, double geneMaximum, int generations, float crossoverChance, float mutationChance, int tournamentSize, float tournamentChance, bool elitism, float fitnessTreshold, SolutionFoundDelegate sf, StoppedDelegate stp, ImprovementDelegate imp, Graphics gReport, Graphics gReport2, Graphics gReport3, Graphics gFitness, Color backColor, Color foreColor1, Color foreColor2)
                {
                    rnd = new Random();
                    Size = size;
                    GenomeLength = genomeLength;
                    GeneMinimum = geneMinimum;
                    GeneMaximum = geneMaximum;
                    Individuals = new Individual[Size];
                    for (int i = 0 ; i < Size ; i++)
                    {
                        Individuals[i] = new Individual(GenomeLength);
                        for (int n = 0 ; n < GenomeLength ; n++)
                            Individuals[i].Genome[n] = (float)(GeneMinimum + rnd.NextDouble() * (GeneMaximum - GeneMinimum));
                    }
                    BestIndividual = new Individual(GenomeLength);

                    Generations = generations;
                    CrossoverChance = crossoverChance;
                    MutationChance = mutationChance;
                    TournamentSize = tournamentSize;
                    TournamentChance = tournamentChance;
                    Elitism = elitism;
                    FitnessTreshold = fitnessTreshold;
                    SF = sf;
                    STP = stp;
                    IMP = imp;
                    GReport = gReport;
                    GReport2 = gReport2;
                    GReport3 = gReport3;
                    GFitness = gFitness;
                    GReportBuffer = new Bitmap(900, 170);
                    GReport2Buffer = new Bitmap(300, 240);
                    GReport3Buffer = new Bitmap(300, 50);
                    GFitnessBuffer = new Bitmap(1260, 250);
                    GReportBufferG = Graphics.FromImage(GReportBuffer);
                    GReport2BufferG = Graphics.FromImage(GReport2Buffer);
                    GReport3BufferG = Graphics.FromImage(GReport3Buffer);
                    GFitnessBufferG = Graphics.FromImage(GFitnessBuffer);
                    GReportBufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                    GReport2BufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                    GReport3BufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                    GFitnessBufferG.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                    BackColor = backColor;
                    ForeColor1 = new SolidBrush(foreColor1);
                    ForeColor2 = new SolidBrush(foreColor2);
                    f = new Font("Arial", 8);
                    f2 = new Font("Arial", 20, FontStyle.Bold);
                    ShowEval = true;
                    ShowImprove = true;
                }

                public void NextGeneration2()
                {
                    int newindividuals = Size / 2;
                    int crossovers = (int)(newindividuals * CrossoverChance / 2);
                    int reproductions = newindividuals - 2 * crossovers;

                    int pos = Size - newindividuals;
                    for (int i = 0 ; i < crossovers ; i++)
                    {
                        Individual[] children = Crossover(Individuals[AGoodChromosomeIndex()], Individuals[AGoodChromosomeIndex()]);
                        Individuals[pos++] = children[0];
                        Individuals[pos++] = children[1];
                    }
                    for (int i = 0 ; i < reproductions ; i++)
                    {
                        Individuals[pos++] = Mutate(Individuals[AGoodChromosomeIndex()]);
                    }
                }

                public void NextGeneration()
                {
                    Individual[] ng = new Individual[Size];

                    int crossovers = (int)(Size * CrossoverChance / 2);
                    int reproductions = Size - 2 * crossovers;

                    int pos = 0;
                    for (int i = 0 ; i < crossovers ; i++)
                    {
                        Individual[] children = Crossover(Individuals[AGoodChromosomeIndex()], Individuals[AGoodChromosomeIndex()]);
                        ng[pos++] = children[0];
                        ng[pos++] = children[1];
                    }
                    for (int i = 0 ; i < reproductions ; i++)
                    {
                        ng[pos++] = Mutate(Individuals[AGoodChromosomeIndex()]);
                    }

                    if (Elitism)
                        ng[0] = BestIndividual;

                    Individuals = ng;
                }



                private int AGoodChromosomeIndex()
                {
                    if (TournamentSize == -1)
                    {
                        double hit = rnd.NextDouble();
                        double pos = 0;
                        for (int i = 0 ; i < Size ; i++)
                        {
                            pos += Individuals[i].Fitness;
                            if (pos >= hit)
                                return i;
                        }
                        return Size - 1;
                    }
                    else
                    {
                        int ret = rnd.Next(Size);
                        int participant;
                        for (int i = 0 ; i < TournamentSize - 1 ; i++)
                        {
                            participant = rnd.Next(Size);
                            if (Individuals[participant].Fitness >= Individuals[ret].Fitness)
                                if (TournamentChance > rnd.NextDouble())
                                    ret = participant;
                        }
                        return ret;
                    }
                }

                private Individual[] Crossover(Individual parent1, Individual parent2)
                {
                    Individual[] children = new Individual[2];
                    children[0] = new Individual(GenomeLength);
                    children[1] = new Individual(GenomeLength);
                    int cut1 = rnd.Next(GenomeLength);
                    int cut2 = rnd.Next(cut1, GenomeLength + 1);
                    for (int i = 0 ; i < cut1 ; i++)
                    {
                        children[0].Genome[i] = parent1.Genome[i];
                        children[1].Genome[i] = parent2.Genome[i];
                    }
                    for (int i = cut1 ; i < cut2 ; i++)
                    {
                        children[0].Genome[i] = parent2.Genome[i];
                        children[1].Genome[i] = parent1.Genome[i];
                    }
                    for (int i = cut2 ; i < GenomeLength ; i++)
                    {
                        children[0].Genome[i] = parent1.Genome[i];
                        children[1].Genome[i] = parent2.Genome[i];
                    }
                    return children;
                }

                private Individual Mutate(Individual i)
                {
                    Individual ret = new Individual(GenomeLength);
                    for (int g = 0 ; g < GenomeLength ; g++)
                    {
                        if (rnd.NextDouble() < MutationChance)
                        {
                            ret.Genome[g] *= (float)(0.8 + 0.4 * rnd.NextDouble());
                            ret.Genome[g] = (float)Math.Max(Math.Min(GeneMaximum, ret.Genome[g]), GeneMinimum);
                        }
                        else
                        {
                            ret.Genome[g] = i.Genome[g];
                        }
                    }
                    return ret;
                }

                internal bool IsEqual(float[] individual1, float[] individual2)
                {
                    for (int i = 0 ; i < GenomeLength ; i++)
                        if (individual1[i] != individual2[i])
                            return false;
                    return true;
                }
            

                public Bitmap GetReportBuffer()
                {
                    return GReportBuffer;
                }

                public Bitmap GetReport2Buffer()
                {
                    return GReport2Buffer;
                }

                public Bitmap GetFitnessBuffer()
                {
                    return GFitnessBuffer;
                }

                public void Evolve()
                {
                    Abort = false;
                    T = new Thread(new ThreadStart(Run));
                    T.Start();
                }

                public void Stop()
                {
                    Abort = true;
                }

                public float[][] GetGenerationBests()
                {
                    if (GenerationBests == null)
                        return new float[0][];
                    float[][] ret = new float[GenerationBests.Count][];
                    GenerationBests.CopyTo(ret);
                    return ret;
                }

                public bool ShowEvaluation
                {
                    get
                    {
                        return ShowEval;
                    }
                    set
                    {
                        ShowEval = value;
                    }
                }

                public bool ShowImprovements
                {
                    get
                    {
                        return ShowImprove;
                    }
                    set
                    {
                        ShowImprove = value;
                    }
                }


                private void Run()
                {
                    DateTime start = DateTime.Now;

                    GReportBufferG.Clear(BackColor);

                    GReportBufferG.DrawString("gbp.AI.GeneticAlgorithms Environment initializing..", f, ForeColor1, 0, 0);
                    GReportBufferG.DrawString("started " + start.ToString(), f, ForeColor2, 0, 10);
                    GReportBufferG.DrawString("population size " + Size.ToString(), f, ForeColor2, 0, 20);
                    GReportBufferG.DrawString("crossover chance " + CrossoverChance.ToString(), f, ForeColor2, 0, 30);
                    GReportBufferG.DrawString("mutation chance " + MutationChance.ToString(), f, ForeColor2, 0, 40);

                    GReport.DrawImage(GReportBuffer, 0, 0);

                    GenerationBests = new ArrayList(100);

                    Graph2D gfitness = new Graph2D(3, GReportBufferG, new RectangleF(280, 5, 200, 140), BackColor, Color.DarkOrange);
                    gfitness.SetComponent(0, "average raw fitness", Color.Orange);
                    gfitness.SetComponent(1, "worst raw fitness", Color.DarkOrange);
                    gfitness.SetComponent(2, "best raw fitness", Color.Yellow);

                    Graph2D gdiversity = new Graph2D(1, GReportBufferG, new RectangleF(490, 5, 200, 140), BackColor, Color.DarkOrange);
                    gdiversity.SetComponent(0, "diversity", Color.Orange);

                    GraphHistogram ghistogram = new GraphHistogram("raw fitness distribution", 15, GReportBufferG, new RectangleF(700, 5, 200, 140), BackColor, Color.DarkOrange, Color.DarkOrange);

                    for (int g = 0 ; g < Generations && !Abort ; g++)
                    {
                        GReport3BufferG.Clear(BackColor);
                        GReport3BufferG.DrawString("GENERATION " + g.ToString(), f2, ForeColor1, 0, 0);
                        GReport3.DrawImage(GReport3Buffer, 0, 0);

                        GiveFitnesses(g);

                        GReportBufferG.Clear(BackColor);

                        GReportBufferG.DrawString("gbp.AI.GeneticAlgorithms Environment running..", f, ForeColor1, 0, 0);
                        GReportBufferG.DrawString("started " + start.ToString(), f, ForeColor2, 0, 10);
                        GReportBufferG.DrawString("population size " + Size.ToString(), f, ForeColor2, 0, 20);
                        GReportBufferG.DrawString("crossover chance " + CrossoverChance.ToString(), f, ForeColor2, 0, 30);
                        GReportBufferG.DrawString("mutation chance " + MutationChance.ToString(), f, ForeColor2, 0, 40);

                        GReportBufferG.DrawString("generation " + g.ToString() + ":", f, ForeColor1, 0, 60);
                        GReportBufferG.DrawString("average raw fitness " + AverageRawFitness.ToString(), f, ForeColor2, 0, 70);
                        GReportBufferG.DrawString("diversity " + Diversity.ToString(), f, ForeColor2, 0, 80);

                        GReportBufferG.DrawString("best individual:", f, ForeColor1, 0, 120);
                        GReportBufferG.DrawString("raw fitness " + Individuals[BestIndex].RawFitness.ToString(), f, ForeColor2, 0, 130);
                        GReportBufferG.DrawString("fitness " + Individuals[BestIndex].Fitness.ToString(), f, ForeColor2, 0, 140);


                        gfitness.AddValue(0, AverageRawFitness);
                        gfitness.AddValue(1, Individuals[WorstIndex].RawFitness);
                        gfitness.AddValue(2, Individuals[BestIndex].RawFitness);
                        gfitness.Draw();

                        gdiversity.AddValue(0, Diversity);
                        gdiversity.Draw();

                        float[] rawfitnesses = new float[Size];
                        for (int i = 0 ; i < Size ; i++)
                            rawfitnesses[i] = Individuals[i].RawFitness;

                        ghistogram.SetValues(rawfitnesses);
                        ghistogram.Draw();

                        GReport.DrawImage(GReportBuffer, 0, 0);

                        GenerationBests.Add(Individuals[BestIndex]);

                        if (Individuals[BestIndex].RawFitness >= FitnessTreshold)
                        {
                            SF(Individuals[BestIndex].Genome, g);
                            break;
                        }

                        NextGeneration();
                    }
                    STP();
                }

                private void GiveFitnesses(int generation)
                {
                    for (int i = 0 ; i < Size && !Abort ; i++)
                    {
                        GReport3BufferG.DrawLine(Pens.Orange, 0, 30, (225f * (i + 1) / Size), 30);
                        GReport3.DrawImage(GReport3Buffer, 0, 0);

                        if (!Individuals[i].FitnessGiven)
                        {
                            Individuals[i].RawFitness = Fitness(Individuals[i].Genome, generation, i, ShowEval);
                            Individuals[i].FitnessGiven = true;
                        }

                        if (Individuals[i].RawFitness > BestIndividual.RawFitness)
                        {
                            BestIndividual = Individuals[i];
                            SF(BestIndividual.Genome, generation);
                            if (ShowImprove)
                            {
                                Fitness(BestIndividual.Genome, generation, i, true);

                            }
                        }

                    }

                    Array.Sort(Individuals);

                    float rawfitnessestotal = 0;
                    for (int i = 0 ; i < Size ; i++)
                    {
                        rawfitnessestotal += Individuals[i].RawFitness;
                        if (Individuals[i].RawFitness > Individuals[BestIndex].RawFitness)
                            BestIndex = i;
                        if (Individuals[i].RawFitness < Individuals[WorstIndex].RawFitness)
                            WorstIndex = i;
                    }
                    AverageRawFitness = rawfitnessestotal / Size;

                    bool unique;
                    float uniques = 0;
                    for (int i = 0 ; i < Size ; i++)
                    {
                        Individuals[i].Fitness = Individuals[i].RawFitness / rawfitnessestotal;

                        unique = true;
                        for (int p = 0 ; p < i ; p++)
                            if (IsEqual(Individuals[i].Genome, Individuals[p].Genome))
                            {
                                unique = false;
                                break;
                            }
                        if (unique)
                            uniques++;
                    }
                    Diversity = uniques / Size;

                }

                public abstract float Fitness(float[] individual, int generation, int index, bool showEvaluation);
                public abstract void DrawIndividual(float[] individual, Graphics g, RectangleF position);

                private class Individual : IComparable
                {
                    public float[] Genome;
                    public float RawFitness;
                    public float Fitness;
                    public bool FitnessGiven;

                    public Individual(int genomeLength)
                    {
                        Genome = new float[genomeLength];
                        RawFitness = 0;
                        Fitness = 0;
                        FitnessGiven = false;
                    }

                    public int CompareTo(object obj)
                    {
                        if (obj is Individual)
                        {
                            Individual i = (Individual)obj;
                            return - RawFitness.CompareTo(i.RawFitness);
                        }
                        return -1;
                   }

                }
            }

            public delegate void SolutionFoundDelegate(float[] solution, int generation);
            public delegate void ImprovementDelegate(string text);
            public delegate void StoppedDelegate();
        }
    }
}
