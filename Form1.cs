using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

namespace PDAContainers
{
    // ==========================================
    // 1. MODELOS
    // ==========================================
    public enum SolutionStatus
    {
        Optimal,
        Unbounded,
        Infeasible,
        IterationLimit
    }

    public class SimplexIteration
    {
        public int IterationNumber { get; set; }
        public double[,] Tableau { get; set; }
        public int PivotRow { get; set; } = -1;
        public int PivotColumn { get; set; } = -1;
        public double PivotValue { get; set; }
        public int[] BasicVariables { get; set; }
        public string Description { get; set; }
    }

    public class SimplexResult
    {
        public SolutionStatus Status { get; set; }
        public double OptimalValue { get; set; }
        public double[] VariableValues { get; set; }
        public double[] SlackValues { get; set; }
        public List<SimplexIteration> History { get; set; } = new List<SimplexIteration>();
    }

    // ==========================================
    // 2. MOTOR DO SIMPLEX
    // ==========================================
    public class SimplexSolver
    {
        public SimplexResult SolveMaximization(double[] objectiveCoeffs, double[,] constraintsCoeffs, double[] rhs)
        {
            int numVars = objectiveCoeffs.Length;
            int numConstraints = rhs.Length;

            int rows = numConstraints + 1;
            int cols = numVars + numConstraints + 1;

            double[,] tableau = new double[rows, cols];
            int[] basicVars = new int[numConstraints];

            for (int i = 0; i < numConstraints; i++)
            {
                for (int j = 0; j < numVars; j++)
                {
                    tableau[i, j] = constraintsCoeffs[i, j];
                }
                tableau[i, numVars + i] = 1.0;
                tableau[i, cols - 1] = rhs[i];
                basicVars[i] = numVars + i;
            }

            for (int j = 0; j < numVars; j++)
            {
                tableau[rows - 1, j] = -objectiveCoeffs[j];
            }

            var result = new SimplexResult();
            int iteration = 0;
            const int maxIterations = 50;

            while (iteration < maxIterations)
            {
                iteration++;

                int pivotCol = GetPivotColumn(tableau, rows, cols);

                if (pivotCol == -1)
                {
                    result.History.Add(CaptureIteration(iteration, tableau, rows, cols, -1, -1, basicVars, "Solução Ótima Atingida"));
                    result.Status = SolutionStatus.Optimal;
                    result.OptimalValue = tableau[rows - 1, cols - 1];
                    ExtractSolution(result, tableau, basicVars, numVars, numConstraints, rows, cols);
                    return result;
                }

                int pivotRow = GetPivotRow(tableau, rows, cols, pivotCol);
                if (pivotRow == -1)
                {
                    result.Status = SolutionStatus.Unbounded;
                    result.History.Add(CaptureIteration(iteration, tableau, rows, cols, -1, pivotCol, basicVars, "Problema Ilimitado"));
                    return result;
                }

                result.History.Add(CaptureIteration(iteration, tableau, rows, cols, pivotRow, pivotCol, basicVars, 
                    $"Entra Coluna {pivotCol + 1}, Sai Linha {pivotRow + 1}"));

                basicVars[pivotRow] = pivotCol;
                Pivot(tableau, rows, cols, pivotRow, pivotCol);
            }

            result.Status = SolutionStatus.IterationLimit;
            return result;
        }

        private int GetPivotColumn(double[,] tableau, int rows, int cols)
        {
            int pivotCol = -1;
            double minVal = -1e-9;

            for (int j = 0; j < cols - 1; j++)
            {
                if (tableau[rows - 1, j] < minVal)
                {
                    minVal = tableau[rows - 1, j];
                    pivotCol = j;
                }
            }
            return pivotCol;
        }

        private int GetPivotRow(double[,] tableau, int rows, int cols, int pivotCol)
        {
            int pivotRow = -1;
            double minRatio = double.MaxValue;

            for (int i = 0; i < rows - 1; i++)
            {
                double elem = tableau[i, pivotCol];
                if (elem > 1e-9)
                {
                    double ratio = tableau[i, cols - 1] / elem;
                    if (ratio < minRatio)
                    {
                        minRatio = ratio;
                        pivotRow = i;
                    }
                }
            }
            return pivotRow;
        }

        private void Pivot(double[,] tableau, int rows, int cols, int pRow, int pCol)
        {
            double pivotElem = tableau[pRow, pCol];

            for (int j = 0; j < cols; j++)
                tableau[pRow, j] /= pivotElem;

            for (int i = 0; i < rows; i++)
            {
                if (i != pRow)
                {
                    double factor = tableau[i, pCol];
                    for (int j = 0; j < cols; j++)
                    {
                        tableau[i, j] -= factor * tableau[pRow, j];
                    }
                }
            }
        }

        private void ExtractSolution(SimplexResult res, double[,] tableau, int[] basicVars, int numVars, int numConstraints, int rows, int cols)
        {
            res.VariableValues = new double[numVars];
            res.SlackValues = new double[numConstraints];

            for (int i = 0; i < numConstraints; i++)
            {
                int varIndex = basicVars[i];
                double value = tableau[i, cols - 1];

                if (varIndex < numVars)
                    res.VariableValues[varIndex] = Math.Round(value, 4);
                else
                    res.SlackValues[varIndex - numVars] = Math.Round(value, 4);
            }
        }

        private SimplexIteration CaptureIteration(int num, double[,] source, int r, int c, int pRow, int pCol, int[] bVars, string desc)
        {
            double[,] copy = new double[r, c];
            Array.Copy(source, copy, source.Length);

            int[] bCopy = new int[bVars.Length];
            Array.Copy(bVars, bCopy, bVars.Length);

            return new SimplexIteration
            {
                IterationNumber = num,
                Tableau = copy,
                PivotRow = pRow,
                PivotColumn = pCol,
                PivotValue = (pRow >= 0 && pCol >= 0) ? source[pRow, pCol] : 0,
                BasicVariables = bCopy,
                Description = desc
            };
        }
    }

    // ==========================================
    // 3. INTERFACE GRÁFICA
    // ==========================================
    public class Form1 : Form
    {
        private NumericUpDown numVarsInput;
        private NumericUpDown numConstraintsInput;
        private Button btnGenerateGrid;
        private Button btnLoadSample;
        private Button btnSolve;
        private DataGridView gridObjective;
        private DataGridView gridConstraints;
        private ListBox listIterations;
        private DataGridView gridTableauViewer;
        private Label lblResult;
        private Label lblSlacks;

        private SimplexResult currentResult;

        public Form1()
        {
            this.Text = "PDA Containers - Otimizador de Cargas e Armazenamento (Simplex)";
            this.Size = new Size(1020, 750);
            this.StartPosition = FormStartPosition.CenterScreen;

            Panel panelTop = new Panel { Dock = DockStyle.Top, Height = 65, Padding = new Padding(10) };

            Label lblVars = new Label { Text = "Variáveis (Cargas):", AutoSize = true, Location = new Point(10, 22) };
            numVarsInput = new NumericUpDown { Location = new Point(125, 19), Width = 50, Minimum = 1, Maximum = 10, Value = 3 };

            Label lblConst = new Label { Text = "Restrições:", AutoSize = true, Location = new Point(185, 22) };
            numConstraintsInput = new NumericUpDown { Location = new Point(255, 19), Width = 50, Minimum = 1, Maximum = 10, Value = 3 };

            btnGenerateGrid = new Button { Text = "Gerar Tabela", Location = new Point(315, 17), Width = 100, Height = 28 };
            btnGenerateGrid.Click += (s, e) => SetupGrids();

            btnLoadSample = new Button { Text = "Carregar Exemplo Real", Location = new Point(425, 17), Width = 155, Height = 28, BackColor = Color.LightSkyBlue };
            btnLoadSample.Click += (s, e) => LoadSampleData();

            btnSolve = new Button { Text = "Calcular Otimização", Location = new Point(590, 17), Width = 145, Height = 28, BackColor = Color.LightGreen };
            btnSolve.Click += (s, e) => SolveProblem();

            panelTop.Controls.AddRange(new Control[] { lblVars, numVarsInput, lblConst, numConstraintsInput, btnGenerateGrid, btnLoadSample, btnSolve });

            gridObjective = new DataGridView { Location = new Point(15, 90), Width = 970, Height = 60, AllowUserToAddRows = false };
            gridConstraints = new DataGridView { Location = new Point(15, 180), Width = 970, Height = 140, AllowUserToAddRows = false };

            lblResult = new Label { Location = new Point(15, 330), AutoSize = true, Font = new Font("Arial", 10, FontStyle.Bold), ForeColor = Color.DarkBlue };
            lblSlacks = new Label { Location = new Point(15, 355), AutoSize = true, Font = new Font("Arial", 9, FontStyle.Regular), ForeColor = Color.DarkSlateGray };

            Label lblStep = new Label { Text = "Histórico de Iterações (Passo a Passo):", Location = new Point(15, 385), AutoSize = true };
            listIterations = new ListBox { Location = new Point(15, 410), Width = 310, Height = 270 };
            listIterations.SelectedIndexChanged += ListIterations_SelectedIndexChanged;

            gridTableauViewer = new DataGridView { Location = new Point(340, 410), Width = 645, Height = 270, ReadOnly = true, AllowUserToAddRows = false };

            this.Controls.AddRange(new Control[] { 
                panelTop, 
                new Label { Text = "Função Objetivo: Maximizar Lucro Z =", Location = new Point(15, 70), AutoSize = true },
                gridObjective, 
                new Label { Text = "Restrições de Capacidade (<= Limite/RHS):", Location = new Point(15, 160), AutoSize = true },
                gridConstraints, 
                lblResult,
                lblSlacks,
                lblStep, 
                listIterations, 
                gridTableauViewer 
            });

            SetupGrids();
        }

        private void SetupGrids()
        {
            int n = (int)numVarsInput.Value;
            int m = (int)numConstraintsInput.Value;

            gridObjective.Columns.Clear();
            gridObjective.Rows.Clear();
            for (int j = 0; j < n; j++)
            {
                gridObjective.Columns.Add($"x{j + 1}", $"Lucro X{j + 1}");
            }
            gridObjective.Rows.Add();

            gridConstraints.Columns.Clear();
            gridConstraints.Rows.Clear();

            // Coluna 0: Nome descritivo da restrição
            var colDesc = new DataGridViewTextBoxColumn { Name = "desc", HeaderText = "Nome da Restrição (Recurso)", Width = 200 };
            gridConstraints.Columns.Add(colDesc);

            for (int j = 0; j < n; j++)
            {
                gridConstraints.Columns.Add($"x{j + 1}", $"Uso X{j + 1}");
            }
            gridConstraints.Columns.Add("rhs", "Limite (RHS)");

            for (int i = 0; i < m; i++)
            {
                int rIdx = gridConstraints.Rows.Add();
                gridConstraints.Rows[rIdx].Cells[0].Value = $"Restrição {i + 1}";
                gridConstraints.Rows[rIdx].HeaderCell.Value = $"R{i + 1}";
            }
        }

        private void LoadSampleData()
        {
            numVarsInput.Value = 3;
            numConstraintsInput.Value = 3;
            SetupGrids();

            // Z = 300*X1 + 700*X2 + 800*X3
            gridObjective.Rows[0].Cells[0].Value = 300;
            gridObjective.Rows[0].Cells[1].Value = 700;
            gridObjective.Rows[0].Cells[2].Value = 800;

            // R1: Peso Máximo do Contentor
            gridConstraints.Rows[0].Cells[0].Value = "Capacidade de Peso (ton)";
            gridConstraints.Rows[0].Cells[1].Value = 2;
            gridConstraints.Rows[0].Cells[2].Value = 4;
            gridConstraints.Rows[0].Cells[3].Value = 5;
            gridConstraints.Rows[0].Cells[4].Value = 60;

            // R2: Volume Útil
            gridConstraints.Rows[1].Cells[0].Value = "Volume Útil (m³)";
            gridConstraints.Rows[1].Cells[1].Value = 3;
            gridConstraints.Rows[1].Cells[2].Value = 3;
            gridConstraints.Rows[1].Cells[3].Value = 4;
            gridConstraints.Rows[1].Cells[4].Value = 50;

            // R3: Orçamento de Manuseio
            gridConstraints.Rows[2].Cells[0].Value = "Orçamento de Manuseio (kR$)";
            gridConstraints.Rows[2].Cells[1].Value = 1;
            gridConstraints.Rows[2].Cells[2].Value = 2;
            gridConstraints.Rows[2].Cells[3].Value = 2;
            gridConstraints.Rows[2].Cells[4].Value = 25;
        }

        private void SolveProblem()
        {
            try
            {
                int n = (int)numVarsInput.Value;
                int m = (int)numConstraintsInput.Value;

                double[] obj = new double[n];
                for (int j = 0; j < n; j++)
                    obj[j] = Convert.ToDouble(gridObjective.Rows[0].Cells[j].Value ?? 0);

                double[,] constraints = new double[m, n];
                double[] rhs = new double[m];
                string[] constraintNames = new string[m];

                for (int i = 0; i < m; i++)
                {
                    constraintNames[i] = gridConstraints.Rows[i].Cells[0].Value?.ToString() ?? $"Restrição {i + 1}";
                    for (int j = 0; j < n; j++)
                    {
                        // Os coeficientes começam na coluna índice 1 devido à coluna de descrição
                        constraints[i, j] = Convert.ToDouble(gridConstraints.Rows[i].Cells[j + 1].Value ?? 0);
                    }

                    rhs[i] = Convert.ToDouble(gridConstraints.Rows[i].Cells[n + 1].Value ?? 0);
                }

                var solver = new SimplexSolver();
                currentResult = solver.SolveMaximization(obj, constraints, rhs);

                if (currentResult.Status == SolutionStatus.Optimal)
                {
                    string varsSummary = "";
                    for (int i = 0; i < currentResult.VariableValues.Length; i++)
                    {
                        varsSummary += $"X{i + 1} = {currentResult.VariableValues[i]} | ";
                    }
                    lblResult.Text = $"Lucro Ótimo Z = R$ {currentResult.OptimalValue:N2} | {varsSummary}";

                    string slacksSummary = "Folgas / Recursos Sobrantes: ";
                    for (int i = 0; i < currentResult.SlackValues.Length; i++)
                    {
                        slacksSummary += $"{constraintNames[i]} = {currentResult.SlackValues[i]} restante | ";
                    }
                    lblSlacks.Text = slacksSummary;
                }
                else
                {
                    lblResult.Text = $"Status: {currentResult.Status}";
                    lblSlacks.Text = "";
                }

                listIterations.Items.Clear();
                foreach (var it in currentResult.History)
                {
                    listIterations.Items.Add($"Iteração {it.IterationNumber}: {it.Description}");
                }

                if (listIterations.Items.Count > 0)
                    listIterations.SelectedIndex = 0;
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Erro na entrada de dados: {ex.Message}", "Aviso", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        private void ListIterations_SelectedIndexChanged(object sender, EventArgs e)
        {
            int idx = listIterations.SelectedIndex;
            if (idx < 0 || currentResult == null || idx >= currentResult.History.Count) return;

            var iter = currentResult.History[idx];
            var tableau = iter.Tableau;
            int rows = tableau.GetLength(0);
            int cols = tableau.GetLength(1);

            gridTableauViewer.Columns.Clear();
            gridTableauViewer.Rows.Clear();

            for (int j = 0; j < cols - 1; j++)
                gridTableauViewer.Columns.Add($"c{j}", $"Col {j + 1}");
            gridTableauViewer.Columns.Add("rhs", "RHS");

            for (int i = 0; i < rows; i++)
            {
                int rIdx = gridTableauViewer.Rows.Add();
                gridTableauViewer.Rows[rIdx].HeaderCell.Value = (i == rows - 1) ? "Linha Z" : $"L{i + 1}";
                for (int j = 0; j < cols; j++)
                {
                    gridTableauViewer.Rows[rIdx].Cells[j].Value = Math.Round(tableau[i, j], 3);
                }
            }//teste

            if (iter.PivotRow >= 0 && iter.PivotColumn >= 0)
            {
                gridTableauViewer.Rows[iter.PivotRow].Cells[iter.PivotColumn].Style.BackColor = Color.Yellow;
            }
        }
    }
}