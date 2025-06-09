
# Data-Driven Observer-Based Model-Free Adaptive Terminal Sliding Mode Control of Rigid Robot Manipulators

This repository contains MATLAB code for simulating the examples presented in our paper:   
📄 **Title**: Data-driven observer-based model-free adaptive discrete-time terminal sliding mode control of rigid robot manipulators  
📰 **Authors**: Babak Esmaeili, Mina Salim, Mahdi Baradarannia, Ali Farzamnia  
📅 **Conference**: Proceedings of the 7th RSI International Conference on Robotics and Mechatronics (ICRoM 2019)
🔗 [DOI: 10.1109/ICRoM48714.2019.9071819](https://doi.org/10.1109/ICRoM48714.2019.9071819)

## 🎯 Overview

This work develops a data-driven observer-based, model-free adaptive terminal sliding mode controller for rigid robot manipulators with unknown dynamics. Key advantages include:
- **Finite-time convergence** via nonlinear terminal sliding surface.
- **Model-free adaptation** using data-driven multi-observers.
- **Robust disturbance rejection** without explicit plant models.

---

## 🧠 Abstract

This paper aims to design a data-driven observer-based model-free adaptive terminal sliding mode controller for rigid robot manipulators whose models are unknown in advance. First, the nonlinear robot manipulator dynamics are transformed to an equivalent linear discrete-time data-model using full-form dynamic linearization technique and multi-observers along with an adaptation law are designed in order to estimate system outputs and pseudo-partitioned Jacobian matrix, respectively. Second, based on a nonlinear data-driven terminal sliding surface, the robust discrete-time controller is obtained. Mathematical analysis guarantees the finite-time convergence and the stability of the closed-loop system. The comparative simulations lighten the superiority of the proposed work and the robustness of the proposed controller against external disturbances is validated by means of simulations.

---

## 🛠 Requirements

- MATLAB R2018b or newer  
- No additional toolboxes are required  

---

## ▶️ Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/Babak-Esmaeili/Data-driven-Observer-based-MFA-TSMC-robot.git
   cd Data-driven-Observer-based-MFA-TSMC-robot/Codes
   ```
2. Open MATLAB and navigate to the `Codes/` folder.  
3. Run `main.m` to reproduce:
   - Joint tracking performance  
   - Control input smoothness  
   - Sliding surface evolution  

---

## 📜 License and Contact

This project is licensed under the MIT License – see [LICENSE](LICENSE).  
For questions or collaboration, contact:

- **Babak Esmaeili** – esmaeil1@msu.edu   

---

## 📚 Citation

If you use this repository, please cite:

```bibtex
@inproceedings{esmaeili2019omfatsmc,
  title={Data-driven observer-based model-free adaptive discrete-time terminal sliding mode control of rigid robot manipulators},
  author={Esmaeili, Babak and Salim, Mina and Baradarannia, Mahdi and Farzamnia, Ali},
  booktitle={7th RSI International Conference on Robotics and Mechatronics (ICRoM)},
  year={2019},
  organization={IEEE}
}
```

---
