#!/bin/bash
# Simple script to convert LaTeX resume to PDF

# Check if pdflatex is installed
if ! command -v pdflatex &> /dev/null; then
    echo "Error: pdflatex is not installed."
    echo "Please install TeX Live or MiKTeX based on your operating system."
    exit 1
fi

# Set the filename
RESUME_FILE="resume"

# Create a directory for temporary files
mkdir -p temp

# Save your LaTeX content to a file
cat > temp/${RESUME_FILE}.tex << 'LATEX_CONTENT'
\documentclass[11pt,a4paper]{article}

% Packages
\usepackage[top=1.5cm, bottom=1.5cm, left=2cm, right=2cm]{geometry}
\usepackage{xcolor}
\usepackage{titlesec}
\usepackage{enumitem}
\usepackage[hidelinks]{hyperref}
\usepackage{fontawesome}
\usepackage{setspace}

% Colors
\definecolor{mainblue}{RGB}{0,102,204}
\definecolor{darkgray}{RGB}{50,50,50}
\definecolor{lightgray}{RGB}{245,245,245}

% Remove paragraph indentation
\setlength{\parindent}{0pt}

% Section formatting
\titleformat{\section}
    {\Large\bfseries\color{mainblue}}
    {}{0em}{}[\vspace{-0.2cm}\rule{\textwidth}{1pt}]
\titlespacing*{\section}{0pt}{*1.2}{*0.8}

% Subsection formatting
\titleformat{\subsection}
    {\bfseries\color{darkgray}}
    {}{0em}{}
\titlespacing*{\subsection}{0pt}{*1.0}{*0.3}

% Remove page numbers
\pagenumbering{gobble}

\begin{document}
\setstretch{1.0}

% Header with name
\begin{minipage}{0.75\textwidth}
    {\LARGE\textbf{\textcolor{mainblue}{Mo Saeidi}}}\\ \vspace{0.1cm}
    {\large\textcolor{darkgray}{Robotics \& Control Engineer}}\\ \vspace{0.3cm}
    
    \begin{tabular}{@{}l@{\ }l@{}}
        \faMapMarker & Via Filippo Corridoni, 22, 20122 Milano MI, Italy\\
        \faPhone & +39 351 365 4158\\
        \faEnvelope & \href{mailto:mohammadhosein.saeidi@mail.polimi.it}{mohammadhosein.saeidi@mail.polimi.it}\\
        \faLinkedin & \href{https://www.linkedin.com/in/mo-saeidi-21a00015a/}{linkedin.com/in/mo-saeidi-21a00015a}\\
        \faGraduationCap & \href{https://scholar.google.com/citations?user=6AucdhQAAAAJ&hl=en}{Google Scholar}\\
    \end{tabular}
\end{minipage}%
\begin{minipage}{0.25\textwidth}
    \raggedleft
    % Placeholder for photo
    % Replace with \includegraphics{photo.jpg} if you have a photo
    \framebox[3.5cm]{\parbox[c][4.2cm][c]{3.5cm}{\centering\textit{Place photo here}\\\textit{(3.5 × 4.2 cm)}}}
\end{minipage}

% Rest of your LaTeX resume content
% ... (copy the rest of your resume here)

\end{document}
LATEX_CONTENT

# Navigate to the temp directory
cd temp

# Compile the LaTeX file twice to ensure references are correct
pdflatex ${RESUME_FILE}.tex
pdflatex ${RESUME_FILE}.tex

# Check if compilation was successful
if [ -f "${RESUME_FILE}.pdf" ]; then
    echo "PDF created successfully!"
    # Copy the PDF to assets/documents
    mkdir -p ../assets/documents
    cp ${RESUME_FILE}.pdf ../assets/documents/
    echo "PDF copied to assets/documents/${RESUME_FILE}.pdf"
else
    echo "Error: PDF creation failed."
fi

# Clean up temporary files if desired
# Uncomment the following line to clean up
# rm -f ${RESUME_FILE}.aux ${RESUME_FILE}.log ${RESUME_FILE}.out ${RESUME_FILE}.tex

# Return to the original directory
cd ..

echo "Done!"