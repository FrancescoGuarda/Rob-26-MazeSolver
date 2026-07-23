---
documentclass: article
classoption:
  - 11pt
  - a4paper
  - oneside
geometry: "top=2cm, bottom=2cm, left=2cm, right=2cm"
linestretch: 1.08
numbersections: true
header-includes:
    - \usepackage[ruled,vlined,linesnumbered]{algorithm2e}
    - \usepackage{amssymb}
    - \usepackage{setspace}
    - \usepackage{float}
    - \usepackage{graphicx}
    - \usepackage{subcaption}
    - '\newcommand{\CapHi}[1]{{\fontsize{14}{16}\selectfont #1}}'
    - \pagestyle{plain}
    - \setcounter{secnumdepth}{3}
    - '\SetKw{Break}{break}'
    - \floatplacement{figure}{H}
    - '\graphicspath{{Immagini/}{report/Immagini/}}'
    - \usepackage{booktabs}
    - \usepackage{adjustbox}
---

\begin{titlepage}
\centering
\includegraphics[width=0.5\textwidth]{res/logo_unibs.png}\par\vspace{1cm}
{\scshape\Large DIPARTIMENTO DI INGEGNERIA DELL'INFORMAZIONE\par}
\vspace{0.3cm}
{\large Corso di Laurea in Ingegneria Informatica\par}
\vspace{2cm}
{\large Relazione del progetto di Robotica\par}
\vspace{0.6cm}
{\LARGE\bfseries Maze Exploration Search Analysis \par}
\vspace{0.8cm}
{\large Anno di Corso 2025-2026\par}
\vfill
\begin{flushleft}
{\large\textbf{Docente del corso:}\par}
{\large Prof. Enrico Scala\par}
{\large Prof. Luigi Gargioni\par}
\end{flushleft}
\vspace{1cm}
\begin{flushright}
{\large\textbf{Studenti:}\par}
{\large Francesco Guarda\par}
{\large Andrea Moro\par}
\end{flushright}
\end{titlepage}

\tableofcontents
\clearpage

# Introduction