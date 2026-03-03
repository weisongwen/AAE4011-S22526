"""
GPS NLOS Classification UI

A Tkinter-based GUI that allows users to:
- Load a GPS NLOS dataset (CSV)
- Select a classification method
- Train and evaluate the model
- View results with visualisations

Author: Dr. Weisong Wen
Department of Aeronautical and Aviation Engineering
The Hong Kong Polytechnic University
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LinearRegression, LogisticRegression
from sklearn.tree import DecisionTreeClassifier
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score, precision_score, recall_score, f1_score


class ClassificationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("GPS NLOS Signal Classification")
        self.root.geometry("1100x750")
        self.root.configure(bg="#f0f0f0")

        self.dataset = None
        self.X_train = self.X_test = self.y_train = self.y_test = None
        self.scaler = StandardScaler()
        self.history = {}  # method_name -> accuracy, for comparison chart

        self._build_ui()

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Header.TLabel", font=("Segoe UI", 14, "bold"), background="#f0f0f0")
        style.configure("TButton", font=("Segoe UI", 10), padding=6)
        style.configure("TLabel", font=("Segoe UI", 10), background="#f0f0f0")
        style.configure("Status.TLabel", font=("Segoe UI", 9), background="#e0e0e0", relief="sunken")

        # --- Header ---
        header = ttk.Label(self.root, text="GPS NLOS Signal Classification Tool", style="Header.TLabel")
        header.pack(pady=(10, 5))

        # --- Control Panel ---
        ctrl = ttk.Frame(self.root, padding=10)
        ctrl.pack(fill="x", padx=10)

        ttk.Button(ctrl, text="Load Dataset (CSV)", command=self._load_dataset).grid(row=0, column=0, padx=5)

        self.file_label = ttk.Label(ctrl, text="No file loaded", width=40)
        self.file_label.grid(row=0, column=1, padx=5)

        ttk.Label(ctrl, text="Method:").grid(row=0, column=2, padx=(15, 5))
        self.method_var = tk.StringVar(value="Logistic Regression")
        method_cb = ttk.Combobox(
            ctrl, textvariable=self.method_var, state="readonly", width=22,
            values=["Linear Regression", "Logistic Regression", "Decision Tree", "SVM (Linear)", "SVM (RBF)"]
        )
        method_cb.grid(row=0, column=3, padx=5)

        ttk.Label(ctrl, text="Test %:").grid(row=0, column=4, padx=(15, 5))
        self.split_var = tk.StringVar(value="20")
        split_cb = ttk.Combobox(
            ctrl, textvariable=self.split_var, state="readonly", width=5,
            values=["10", "20", "30", "40"]
        )
        split_cb.grid(row=0, column=5, padx=5)

        ttk.Button(ctrl, text="Run Classification", command=self._run_classification).grid(row=0, column=6, padx=(15, 5))

        # --- Main Area (results text + figure) ---
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)

        # Left: results text
        left = ttk.LabelFrame(main_frame, text="Results", padding=5)
        left.pack(side="left", fill="both", expand=False, padx=(0, 5))

        self.result_text = tk.Text(left, width=42, height=35, font=("Consolas", 9), wrap="word")
        scrollbar = ttk.Scrollbar(left, orient="vertical", command=self.result_text.yview)
        self.result_text.configure(yscrollcommand=scrollbar.set)
        self.result_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Right: figure canvas
        right = ttk.LabelFrame(main_frame, text="Visualisation", padding=5)
        right.pack(side="right", fill="both", expand=True)

        self.fig, self.axes = plt.subplots(1, 2, figsize=(7, 3.5))
        self.fig.tight_layout(pad=2.5)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # --- Dataset Preview ---
        preview_frame = ttk.LabelFrame(self.root, text="Dataset Preview", padding=5)
        preview_frame.pack(fill="x", padx=10, pady=(0, 5))

        self.tree = ttk.Treeview(preview_frame, height=5, show="headings")
        tree_scroll = ttk.Scrollbar(preview_frame, orient="horizontal", command=self.tree.xview)
        self.tree.configure(xscrollcommand=tree_scroll.set)
        self.tree.pack(fill="x", expand=True)
        tree_scroll.pack(fill="x")

        # --- Status Bar ---
        self.status_var = tk.StringVar(value="Ready. Load a CSV dataset to begin.")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, style="Status.TLabel")
        status_bar.pack(fill="x", padx=10, pady=(0, 10), ipady=3)

    def _load_dataset(self):
        path = filedialog.askopenfilename(
            title="Select GPS NLOS Dataset",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not path:
            return

        try:
            self.dataset = pd.read_csv(path)
            self.file_label.config(text=path.split("/")[-1].split("\\")[-1])
            self._show_preview()
            self.status_var.set(f"Loaded {len(self.dataset)} rows, {len(self.dataset.columns)} columns.")
            self.result_text.delete("1.0", tk.END)
            self.result_text.insert(tk.END, f"Dataset loaded: {len(self.dataset)} samples\n")
            self.result_text.insert(tk.END, f"Columns: {', '.join(self.dataset.columns)}\n\n")
            self.result_text.insert(tk.END, f"Class distribution:\n{self.dataset['NLOS_Status'].value_counts().to_string()}\n")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load dataset:\n{e}")

    def _show_preview(self):
        self.tree.delete(*self.tree.get_children())
        cols = list(self.dataset.columns)
        self.tree["columns"] = cols
        for c in cols:
            self.tree.heading(c, text=c)
            self.tree.column(c, width=100, anchor="center")
        for _, row in self.dataset.head(5).iterrows():
            self.tree.insert("", "end", values=[f"{v:.4f}" if isinstance(v, float) else str(v) for v in row])

    def _run_classification(self):
        if self.dataset is None:
            messagebox.showwarning("No Data", "Please load a dataset first.")
            return

        self.result_text.delete("1.0", tk.END)
        method = self.method_var.get()
        test_size = int(self.split_var.get()) / 100

        try:
            features = [c for c in self.dataset.columns if c != "NLOS_Status"]
            X = self.dataset[features].values
            y = self.dataset["NLOS_Status"].values

            self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(
                X, y, test_size=test_size, random_state=42, stratify=y
            )
            X_train_s = self.scaler.fit_transform(self.X_train)
            X_test_s = self.scaler.transform(self.X_test)

            model, is_proba = self._get_model(method)
            model.fit(X_train_s, self.y_train)

            if method == "Linear Regression":
                y_pred_raw = model.predict(X_test_s)
                y_pred = (y_pred_raw > 0.5).astype(int)
            else:
                y_pred = model.predict(X_test_s)

            acc = accuracy_score(self.y_test, y_pred)
            prec = precision_score(self.y_test, y_pred)
            rec = recall_score(self.y_test, y_pred)
            f1 = f1_score(self.y_test, y_pred)
            n_correct = int((y_pred == self.y_test).sum())
            n_wrong = len(self.y_test) - n_correct

            self.history[method] = acc

            self._display_results(method, acc, prec, rec, f1, n_correct, n_wrong)
            self._plot_results(method, acc, n_correct, n_wrong)
            self.status_var.set(f"{method} — Accuracy: {acc:.2%}")

        except Exception as e:
            messagebox.showerror("Error", f"Classification failed:\n{e}")

    def _get_model(self, method):
        if method == "Linear Regression":
            return LinearRegression(), False
        elif method == "Logistic Regression":
            return LogisticRegression(max_iter=1000, random_state=42), True
        elif method == "Decision Tree":
            return DecisionTreeClassifier(max_depth=5, random_state=42), True
        elif method == "SVM (Linear)":
            return SVC(kernel="linear", probability=True, random_state=42), True
        elif method == "SVM (RBF)":
            return SVC(kernel="rbf", probability=True, random_state=42), True

    def _display_results(self, method, acc, prec, rec, f1, n_correct, n_wrong):
        self.result_text.insert(tk.END, f"{'='*40}\n")
        self.result_text.insert(tk.END, f"  Method: {method}\n")
        self.result_text.insert(tk.END, f"{'='*40}\n\n")
        self.result_text.insert(tk.END, f"Total test samples:  {n_correct + n_wrong}\n")
        self.result_text.insert(tk.END, f"Correct predictions: {n_correct}\n")
        self.result_text.insert(tk.END, f"Wrong predictions:   {n_wrong}\n\n")
        self.result_text.insert(tk.END, f"Accuracy:   {acc:.2%}\n")
        self.result_text.insert(tk.END, f"Precision:  {prec:.2%}\n")
        self.result_text.insert(tk.END, f"Recall:     {rec:.2%}\n")
        self.result_text.insert(tk.END, f"F1 Score:   {f1:.2%}\n")

        if len(self.history) > 1:
            self.result_text.insert(tk.END, f"\n{'─'*40}\n")
            self.result_text.insert(tk.END, "  Accuracy Comparison (all runs)\n")
            self.result_text.insert(tk.END, f"{'─'*40}\n")
            for name, a in self.history.items():
                marker = " ◀" if name == method else ""
                self.result_text.insert(tk.END, f"  {name:<22} {a:.2%}{marker}\n")

    def _plot_results(self, method, acc, n_correct, n_wrong):
        for ax in self.axes:
            ax.clear()

        # Left: Correct vs Wrong bar chart for current run
        ax0 = self.axes[0]
        bars = ax0.bar(["Correct", "Wrong"], [n_correct, n_wrong],
                       color=["#4CAF50", "#F44336"], width=0.5, edgecolor="white")
        ax0.set_title(f"{method}", fontsize=10)
        ax0.set_ylabel("Number of Predictions", fontsize=9)
        for bar in bars:
            h = bar.get_height()
            ax0.text(bar.get_x() + bar.get_width() / 2, h + 1,
                     str(int(h)), ha="center", va="bottom", fontsize=11, fontweight="bold")
        ax0.set_ylim(0, max(n_correct, n_wrong) * 1.2)

        # Right: Accuracy comparison across all methods run so far
        ax1 = self.axes[1]
        names = list(self.history.keys())
        accs = [self.history[n] * 100 for n in names]
        colors = ["#2196F3" if n != method else "#FF9800" for n in names]
        bars = ax1.barh(names, accs, color=colors, height=0.5, edgecolor="white")
        ax1.set_title("Accuracy Comparison (%)", fontsize=10)
        ax1.set_xlim(0, 105)
        for bar, a in zip(bars, accs):
            ax1.text(bar.get_width() + 0.8, bar.get_y() + bar.get_height() / 2,
                     f"{a:.1f}%", va="center", fontsize=9, fontweight="bold")
        ax1.tick_params(axis="y", labelsize=8)

        self.fig.tight_layout(pad=2.5)
        self.canvas.draw()


if __name__ == "__main__":
    root = tk.Tk()
    app = ClassificationApp(root)
    root.mainloop()
