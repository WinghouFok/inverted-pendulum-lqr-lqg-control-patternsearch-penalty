from pathlib import Path
import math

import pandas as pd
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle


ROOT = Path(__file__).resolve().parents[1]
LOG_PATH = ROOT / "results" / "cart_pendulum_practical_log.csv"
WEIGHTS_PATH = ROOT / "results" / "optimized_servo_state_weights.csv"
PDF_PATH = ROOT / "docs" / "ieee_report.pdf"

if not LOG_PATH.exists():
    LOG_PATH = ROOT / "cart_pendulum_practical_log.csv"
if not WEIGHTS_PATH.exists():
    WEIGHTS_PATH = ROOT / "optimized_servo_state_weights.csv"


def styles():
    base = getSampleStyleSheet()
    base.add(ParagraphStyle(
        name="TitleIEEE",
        parent=base["Title"],
        fontName="Times-Bold",
        fontSize=18,
        leading=21,
        alignment=TA_CENTER,
    ))
    base.add(ParagraphStyle(
        name="AuthorIEEE",
        parent=base["Normal"],
        fontName="Times-Roman",
        fontSize=10,
        leading=12,
        alignment=TA_CENTER,
        spaceAfter=10,
    ))
    base.add(ParagraphStyle(
        name="BodyIEEE",
        parent=base["Normal"],
        fontName="Times-Roman",
        fontSize=9,
        leading=10.5,
        alignment=TA_JUSTIFY,
        spaceAfter=5,
    ))
    base.add(ParagraphStyle(
        name="SectionIEEE",
        parent=base["Heading2"],
        fontName="Times-Bold",
        fontSize=10,
        leading=12,
        alignment=TA_CENTER,
        spaceBefore=8,
        spaceAfter=4,
    ))
    return base


def metric_summary():
    df = pd.read_csv(LOG_PATH)
    theta_deg = df["theta"] * 180 / math.pi
    err = df["tracking_error"]
    theta_est = df["theta_est_error_deg"]
    xc_est = df["xc_est_error"]
    return {
        "samples": len(df),
        "duration": df["t"].iloc[-1],
        "max_theta_deg": theta_deg.abs().max(),
        "max_u_actual": df["u_actual"].abs().max(),
        "tracking_rmse": math.sqrt((err * err).mean()),
        "final_error": err.iloc[-1],
        "rms_theta_est_deg": math.sqrt((theta_est * theta_est).mean()),
        "rms_xc_est": math.sqrt((xc_est * xc_est).mean()),
        "sat_percent": 100 * (df["u_cmd_unsat"].abs() > 20).mean(),
    }


def make_table(rows, widths):
    tbl = Table(rows, colWidths=widths, hAlign="LEFT")
    tbl.setStyle(TableStyle([
        ("FONT", (0, 0), (-1, -1), "Times-Roman", 8),
        ("FONT", (0, 0), (-1, 0), "Times-Bold", 8),
        ("LINEABOVE", (0, 0), (-1, 0), 0.6, colors.black),
        ("LINEBELOW", (0, 0), (-1, 0), 0.6, colors.black),
        ("LINEBELOW", (0, -1), (-1, -1), 0.6, colors.black),
        ("ALIGN", (1, 1), (-1, -1), "RIGHT"),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
    ]))
    return tbl


def main():
    st = styles()
    metrics = metric_summary()
    weights = pd.read_csv(WEIGHTS_PATH)
    story = [
        Paragraph("Observer-Based LQR/LQG Control of an Inverted Pendulum on a Cart", st["TitleIEEE"]),
        Paragraph("Yonghao Huo", st["AuthorIEEE"]),
        Paragraph("<b>Abstract</b>--This report presents the design and nonlinear simulation of an observer-based servo controller for an inverted pendulum mounted on a cart. A linearized state-space model is used to design an LQR controller with integral action for cart-position tracking. A Kalman/LQE observer estimates unmeasured velocity states from noisy measurements. The controller is validated on a nonlinear plant with actuator limits, friction, disturbances, and sensor noise.", st["BodyIEEE"]),
        Paragraph("<b>Index Terms</b>--Inverted pendulum, LQR, LQG, Kalman observer, nonlinear simulation, pattern search.", st["BodyIEEE"]),
        Paragraph("I. INTRODUCTION", st["SectionIEEE"]),
        Paragraph("The inverted pendulum on a cart is a classical benchmark problem because it is nonlinear and unstable around the upright equilibrium. This project demonstrates a full control workflow: modeling, observer-based feedback design, automatic LQR weight tuning, nonlinear validation, logging, and animation.", st["BodyIEEE"]),
        Paragraph("II. METHOD", st["SectionIEEE"]),
        Paragraph("The state vector is x = [theta, theta_dot, x_c, x_c_dot]^T. A servo LQR controller is designed with an additional integral state for cart-position tracking. A Kalman/LQE observer estimates unmeasured velocity states from noisy measurements of pendulum angle and cart position.", st["BodyIEEE"]),
        Paragraph("The practical simulation includes actuator saturation, slew-rate limiting, first-order actuator lag, cart friction, measurement noise, disturbance pulses, anti-windup logic, and safety limits. The nonlinear plant is integrated using fourth-order Runge-Kutta integration.", st["BodyIEEE"]),
        Paragraph("III. AUTOMATIC WEIGHT TUNING", st["SectionIEEE"]),
        Paragraph("The LQR state weights are searched in log space using MATLAB patternsearch when available, with a local pattern-search fallback. Candidate controllers are scored over multiple nonlinear cases using a penalty objective based on angle, tracking error, velocity, actuator effort, saturation, settling behavior, and safety violations.", st["BodyIEEE"]),
        make_table([["State", "Optimized Weight"]] + [[str(r["State"]), f"{float(r['OptimizedWeight']):.4g}"] for _, r in weights.iterrows()], [2.0 * inch, 1.5 * inch]),
        Paragraph("IV. RESULTS", st["SectionIEEE"]),
        Paragraph(f"The saved simulation contains {metrics['samples']} samples over {metrics['duration']:.0f} seconds. The controller stabilizes the pendulum, tracks the smooth reference, and handles disturbance pulses without a safety stop.", st["BodyIEEE"]),
        make_table([
            ["Metric", "Value"],
            ["Maximum pendulum angle", f"{metrics['max_theta_deg']:.2f} deg"],
            ["Maximum actual actuator force", f"{metrics['max_u_actual']:.2f} N"],
            ["Tracking RMSE", f"{metrics['tracking_rmse']:.3f} m"],
            ["Final tracking error", f"{metrics['final_error']:.4f} m"],
            ["RMS theta estimation error", f"{metrics['rms_theta_est_deg']:.3f} deg"],
            ["RMS cart-position estimation error", f"{metrics['rms_xc_est']:.5f} m"],
            ["Force saturation percentage", f"{metrics['sat_percent']:.2f}%"],
        ], [2.5 * inch, 1.4 * inch]),
        Paragraph("V. CONCLUSION", st["SectionIEEE"]),
        Paragraph("This project connects control theory with practical implementation issues such as state estimation, actuator constraints, disturbance rejection, and nonlinear validation. Future improvements could compare LQR against PID or MPC, add robustness analysis, or implement the controller in Simulink.", st["BodyIEEE"]),
        Paragraph("REFERENCES", st["SectionIEEE"]),
        Paragraph("[1] K. J. Astrom and R. M. Murray, Feedback Systems: An Introduction for Scientists and Engineers, Princeton University Press, 2008.", st["BodyIEEE"]),
        Paragraph("[2] K. Ogata, Modern Control Engineering, 5th ed., Prentice Hall, 2010.", st["BodyIEEE"]),
    ]
    PDF_PATH.parent.mkdir(exist_ok=True)
    doc = SimpleDocTemplate(str(PDF_PATH), pagesize=letter, leftMargin=0.65 * inch, rightMargin=0.65 * inch, topMargin=0.7 * inch, bottomMargin=0.7 * inch)
    doc.build(story)
    print(f"Wrote {PDF_PATH}")


if __name__ == "__main__":
    main()
