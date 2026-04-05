# FOC Auto-Calibration and Motor Control Reference Bibliography

This document compiles the 10+ most significant academic and technical papers on FOC auto-tuning, PI gain tuning, sensorless control, field weakening, and cascaded loop robustness for BLDC/PMSM motors.

---

## Core FOC Implementation and Auto-Tuning

### 1. **MCUXpresso SDK Field-Oriented Control of 3-Phase PMSM and BLDC Motors**
- **Source**: NXP Semiconductors (User Guide UG10261)
- **URL**: https://www.nxp.com/docs/en/user-guide/UG10261.pdf
- **Key Contribution**: Comprehensive FOC implementation guide for PMSM and BLDC motors including the Motor Control Application Tuning (MCAT) plugin for automatic parameter calibration and sensor offset computation.
- **Relevance**: Directly applicable to auto-calibration module development. Describes practical auto-tuning and auto-calibration implementation in production motor control systems.

### 2. **Closed-Loop PID Autotuner Block - MATLAB & Simulink**
- **Source**: MathWorks Documentation
- **URL**: https://www.mathworks.com/help/slcontrol/ug/tune-field-oriented-controllers-using-closed-loop-pid-autotuner-block.html
- **Key Contribution**: Describes the closed-loop autotuner block that injects sinusoidal perturbation signals at plant input and measures output response, then computes PID gains based on estimated frequency response. Frequency points sampled at [1/10, 1/3, 1, 3, 10]×ωc where ωc is target bandwidth.
- **Relevance**: Critical for understanding bandwidth-based PI tuning methodology. The perturbation approach and frequency response estimation provide a proven methodology for adaptive tuning.

---

## PI Gain Tuning Methods (Bandwidth-Based and Model-Based)

### 3. **Current Loop Tuning - Microchip MCAF R7 RC37 Documentation**
- **Source**: Microchip Technology (MCAF Documentation)
- **URL**: https://microchiptech.github.io/mcaf-doc/7.0.2/algorithms/foc/tuning.html
- **Key Contribution**: Detailed current loop tuning guidelines including bandwidth calculation as R/L ratio, PI gain selection based on phase margin (typical 45-80°), and practical formulas for Kp/Ki computation from motor parameters.
- **Relevance**: Essential reference for analytical current loop bandwidth calculation. Provides the R/L bandwidth formula and phase margin-based tuning rules directly applicable to the auto-tuning module.

### 4. **High-Performance Current Loop Control Design - DigiKey**
- **Source**: DigiKey Electronics Technical Article
- **URL**: https://www.digikey.com/en/articles/an-easier-approach-to-high-performance-current-loop-design
- **Key Contribution**: Practical guidelines for current loop bandwidth: 5-15% of PWM frequency (typically 1-3 kHz at 20 kHz PWM). Explains phase margin selection (0-10% overshoot), and constraints on achievable bandwidth.
- **Relevance**: Provides practical operational constraints and empirical bandwidth targets for validation of auto-tuned gains.

### 5. **High Bandwidth Current Control of 3-Phase PMSM - TI Application Report**
- **Source**: Texas Instruments (SPRAC55)
- **URL**: https://www.ti.com/lit/pdf/sprac55
- **Key Contribution**: Details high-bandwidth current control design with frequency sweep correction methodology. Achieves 5+ kHz bandwidth with careful gain tuning and compensation.
- **Relevance**: Demonstrates advanced current loop design for the demanding 4000 RPM motors (Motenergy series) in the SPINOTOR project.

### 6. **Simplified Gain and Phase Margin PI Tuning Method for SPMSM Control**
- **Source**: IEEE Xplore Conference Publication (2023)
- **URL**: https://ieeexplore.ieee.org/document/9906536
- **Key Contribution**: Proposes the SGPM method which simplifies the gain and phase margin (GPM) tuning approach for surface-mounted PMSM. Provides closed-form solutions for PI gains based on specified phase margin.
- **Relevance**: Offers simplified analytical formulation for PI tuning that reduces computational complexity compared to full frequency-domain methods.

### 7. **Tuning Rules for PI Gains of Field-Oriented Controllers of Induction Motors**
- **Source**: IEEE Transactions (Classic Paper - IEEE Xplore)
- **URL**: https://ieeexplore.ieee.org/document/847900
- **Key Contribution**: Foundational work on PI tuning rules using frequency-domain methods and stability boundary locus. Provides theoretical basis for gain scheduling and parameter-dependent tuning.
- **Relevance**: Theoretical foundation for understanding the relationship between motor parameters and required PI gains.

### 8. **A Simple Tuning Method of PI Regulators in FOC for PMSM Drives Based on Deadbeat Predictive Conception**
- **Source**: IEEE Transactions (2024)
- **URL**: https://ieeexplore.ieee.org/document/10491327
- **Key Contribution**: Presents deadbeat predictive control as an alternative to traditional PI-based FOC. Offers superior dynamic response but requires high model accuracy. Includes adaptive flux-weakening controller simplification.
- **Relevance**: Provides alternative control architecture and validation method for comparing PI-based and predictive control performance metrics.

---

## Sensorless Control and Startup Strategies

### 9. **Sensorless Field-Oriented Control for PMSM Using Sliding Mode Observer**
- **Source**: Microchip Application Note AN4398
- **URL**: https://ww1.microchip.com/downloads/aemDocuments/documents/MCU32/ApplicationNotes/ApplicationNotes/AN4398-Sensorless-Field-Oriented-Control-for-a-Permanent-Magnet-Synchronous-Motor-Using-Sliding-Mode-Observer-DS00004398.pdf
- **Key Contribution**: Complete sensorless FOC implementation using SMO with back-EMF estimation. Covers startup procedures (I/f ramp method) and transition to sensorless control at medium speeds.
- **Relevance**: Provides practical implementation framework for sensorless startup that can be integrated with auto-calibration module to validate operation across full speed range.

### 10. **I/f Startup Method for Sensorless PMSM Control**
- **Source**: Imperix Technical Documentation
- **URL**: https://imperix.com/doc/implementation/i-f-startup-method
- **Key Contribution**: Describes the I/f (Current-Frequency) open-loop startup method. Motor accelerates with constant torque (constant Iq) while rotor frequency ramps up. Smooth transition to flux observer at medium speeds.
- **Relevance**: Critical for validating auto-tuned gains during motor startup phase, ensuring current control performance at zero speed conditions.

### 11. **SMO-Based Sensorless Control of PMSM - Frontiers in Energy Research**
- **Source**: Frontiers (Open Access - PMC/Springer)
- **URL**: https://www.frontiersin.org/journals/energy-research/articles/10.3389/fenrg.2022.839329/full
- **Key Contribution**: Comprehensive review of sliding-mode observer (SMO) techniques for sensorless PMSM control. Addresses robustness to parameter variations and nonlinearities inherent in SMO design.
- **Relevance**: Reviews observer tuning methods that interact with cascaded PI control and current loop bandwidth requirements.

### 12. **Sensorless Field Oriented Control of CSI-Fed PMSM**
- **Source**: arXiv Preprint (2025)
- **URL**: https://www.arxiv.org/pdf/2503.22855
- **Key Contribution**: Recent research on sensorless FOC for current-source inverter fed PMSM. Includes stability analysis and control structure validation.
- **Relevance**: Provides latest approaches and stability verification methods for sensorless control validation.

---

## Parameter Identification and Adaptive Tuning

### 13. **Parameter Identification of PMSM Based on LSOSMO Algorithm**
- **Source**: PMC/NIH Journal (2024)
- **URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC12074470/
- **Key Contribution**: Describes least-squares with online SMO algorithm for real-time motor parameter identification. Simultaneously estimates Rs, Ls, Lq, Ψm with convergence guarantees.
- **Relevance**: Provides online parameter identification methodology to improve auto-calibration accuracy when motor parameters are unknown or temperature-dependent.

### 14. **Online Intelligent Parameter and Speed Estimation of PMSM Using Bacterial Foraging Optimization**
- **Source**: STET Review (2025, Open Access)
- **URL**: https://www.stet-review.org/articles/stet/full_html/2025/01/stet20250054/stet20250054.html
- **Key Contribution**: Proposes bio-inspired optimization (BFO) for real-time parameter estimation. Handles parameter uncertainty and adapts to operating conditions.
- **Relevance**: Demonstrates advanced adaptive identification methods for improving robustness of auto-tuning under varying load and temperature conditions.

### 15. **UKF-Based Parameter Estimation and Identification for PMSM**
- **Source**: Frontiers (Open Access)
- **URL**: https://www.frontiersin.org/articles/10.3389/fenrg.2022.855649/full
- **Key Contribution**: Unscented Kalman Filter approach for simultaneous parameter and state estimation. Non-linear estimation without explicit motor model inversion.
- **Relevance**: Alternative statistical approach to parameter identification with formal optimality guarantees (minimum variance unbiased estimator).

### 16. **A Novel Online Multivariate Identification for Autotuning Speed Control in PMSM Drives**
- **Source**: Mathematical Problems in Engineering, Wiley (2016)
- **URL**: https://onlinelibrary.wiley.com/doi/10.1155/2016/1780710
- **Key Contribution**: Describes multivariate online identification technique for joint estimation of rotor inertia, friction, and load torque. Enables gain scheduling with operating point adaptation.
- **Relevance**: Extends parameter identification beyond electrical parameters to mechanical load estimation, improving speed loop tuning robustness.

---

## Field Weakening Control

### 17. **Field-Weakening Control with MTPA - MATLAB & Simulink Example**
- **Source**: MathWorks Documentation
- **URL**: https://www.mathworks.com/help/mcb/gs/field-weakening-control-mtpa-pmsm.html
- **Key Contribution**: Describes field-weakening implementation with maximum torque per ampere (MTPA) operation. Explains d-axis current modulation to maintain voltage constraint while extending speed range.
- **Relevance**: Essential for validating auto-tuned PI gains across field-weakening region where motor dynamics change significantly.

### 18. **FIELD WEAKENING CONTROL OF PMSM - AAU ETD Thesis**
- **Source**: Addis Ababa University (Academic Thesis)
- **URL**: https://etd.aau.edu.et/bitstreams/a86f7769-5d26-4d34-a3ff-7464ef3cd1d3/download
- **Key Contribution**: Comprehensive treatment of field-weakening control design. Covers voltage and current constraints, operating region boundaries, and controller design in field-weakening mode.
- **Relevance**: Provides theoretical framework for understanding how PI gains must be adjusted or scheduled across different operating regions.

### 19. **Field Weakening Operation Control Strategies Based on Feedback Linearization - MDPI**
- **Source**: Energies Journal, MDPI (2019)
- **URL**: https://www.mdpi.com/1996-1073/12/23/4526
- **Key Contribution**: Analyzes field-weakening control using feedback linearization and Lyapunov stability analysis. Provides robust design methodology.
- **Relevance**: Theoretical foundation for robust field-weakening control design with stability guarantees.

---

## Cascaded Control and Anti-Windup

### 20. **Protection of Cascaded Loops Against Windup and Limit Cycling - Advanced Control for Applications**
- **Source**: Wiley Online Library (2025)
- **URL**: https://onlinelibrary.wiley.com/doi/full/10.1002/adc2.70027
- **Key Contribution**: Comprehensive analysis of cascade-type integral windup in multi-loop control structures. Compares back-calculation, clamping, and tracking anti-windup methods.
- **Relevance**: Critical for FOC implementation with cascaded current and speed loops. Prevents performance degradation when inner loop saturates.

### 21. **Anti-Windup PID Controller With Integral State Predictor for Variable-Speed Motor Drives**
- **Source**: ResearchGate/Academic Publication
- **URL**: https://www.researchgate.net/publication/252063153
- **Key Contribution**: Proposes integral state predictor method for anti-windup. Decouples PI integrator from saturation effects in cascaded motor drive control.
- **Relevance**: Provides practical anti-windup implementation strategy specifically designed for motor drive applications.

### 22. **High Order Disturbance Observer Based PI-PI Control System With Tracking Anti-Windup for PMSM**
- **Source**: ResearchGate (2021)
- **URL**: https://www.researchgate.net/publication/351434881
- **Key Contribution**: Combines high-order disturbance observation with PI-PI cascade control and tracking anti-windup. Improves transient performance and load disturbance rejection.
- **Relevance**: Demonstrates integration of anti-windup with advanced observer techniques for robust cascaded control design.

### 23. **Antiwindup Design for Speed Loop PI Controller of PMSM Servo System**
- **Source**: ResearchGate/Academic Publication
- **URL**: https://www.researchgate.net/publication/272700865
- **Key Contribution**: Focuses specifically on anti-windup design for outer speed loop PI controller in cascaded FOC structure.
- **Relevance**: Addresses outer loop saturation effects that directly impact auto-tuned PI performance validation.

### 24. **Robust μ Parameterization with Low Tuning Complexity of Cascaded Control for Feed Drives**
- **Source**: ScienceDirect (2023)
- **URL**: https://sciencedirect.com/science/article/pii/S0967066123001764
- **Key Contribution**: Robust control design using structured singular value (μ) analysis for cascaded control systems. Provides tuning methodology with guaranteed robustness margins.
- **Relevance**: Theoretical framework for robust multi-loop control design with quantified robustness guarantees.

### 25. **Robust Fractional-Order PI/PD Controllers for Cascade Control Structure**
- **Source**: MDPI Fractal and Fractional (2024)
- **URL**: https://www.mdpi.com/2504-3110/8/4/244
- **Key Contribution**: Extends PI control to fractional-order (FoPID) in cascaded structures. Provides additional tuning parameter (order α) for improved disturbance rejection and robustness.
- **Relevance**: Advanced control technique that can improve upon integer-order PI limitations in cascaded loops.

---

## Sensorless Control Robustness and Stability

### 26. **Robust Sensorless PMSM Control with Improved Back-EMF Observer and Adaptive Parameter Estimation - MDPI**
- **Source**: Electronics Journal, MDPI (2025)
- **URL**: https://www.mdpi.com/2079-9292/14/7/1238
- **Key Contribution**: Proposes improved back-EMF extended state observer (ESO) with adaptive RLS parameter identification. Compensates for DC position error without increasing observer bandwidth.
- **Relevance**: Addresses observer tuning interaction with PI current loop bandwidth. Shows how parameter adaptation improves robustness.

### 27. **Sensorless Control of PMSM Based on Fuzzy Sliding Mode Observer and Terminal Sliding Mode Control - MDPI**
- **Source**: Applied Sciences Journal, MDPI (2025)
- **URL**: https://www.mdpi.com/2076-3417/16/5/2544
- **Key Contribution**: Combines fuzzy logic with sliding mode observer for robust sensorless control. Handles parameter uncertainties without explicit parameter knowledge.
- **Relevance**: Demonstrates non-linear control integration with sensorless operation, improving robustness against motor parameter variations.

### 28. **Sensorless Control of Ultra-High-Speed PMSM via Improved PR and Adaptive Position Observer - MDPI**
- **Source**: Sensors Journal, MDPI (2025)
- **URL**: https://www.mdpi.com/1424-8220/25/5/1290
- **Key Contribution**: Proportional-resonant (PR) current control with adaptive observer for ultra-high-speed operation. Extends sensorless control bandwidth beyond traditional PI limitations.
- **Relevance**: Shows advanced current control alternatives and demonstrates bandwidth extension techniques applicable to high-speed motor operation.

### 29. **Stability Analysis of PI-Controller-Type Position Estimator for Sensorless PMSM Drives in Flux Weakening Region**
- **Source**: IEEE Xplore (2021)
- **URL**: https://ieeexplore.ieee.org/document/9490071
- **Key Contribution**: Stability analysis of sensorless control specifically in field-weakening region. Identifies parameter ranges for stable operation across extended speed range.
- **Relevance**: Critical for validating auto-tuned gains in field-weakening operating region where control dynamics are significantly altered.

### 30. **Stability Analysis of I-f Startup Method for PMSM Drive Systems**
- **Source**: Journal of Measurement, Control and Automation
- **URL**: https://mca-journal.org/index.php/mca/article/view/209
- **Key Contribution**: Theoretical stability analysis of I/f startup transition. Identifies critical speed threshold and acceleration rate constraints.
- **Relevance**: Provides theoretical framework for validating auto-tuned current loop during startup phase transitions.

---

## Advanced Control Techniques and Learning-Based Approaches

### 31. **Reinforcement Learning for Motor Control: A Comprehensive Review**
- **Source**: arXiv Preprint (2024)
- **URL**: https://arxiv.org/html/2412.17936v1
- **Key Contribution**: Survey of reinforcement learning applications in motor control. Reviews DQN, DDPG, TD3, and actor-critic methods for learning optimal control policies without explicit models.
- **Relevance**: Provides alternative data-driven approach to PI tuning that could supplement or replace traditional optimization-based auto-calibration.

### 32. **Tune PI Controller Using Reinforcement Learning - MATLAB & Simulink**
- **Source**: MathWorks Documentation
- **URL**: https://www.mathworks.com/help/reinforcement-learning/ug/tune-pi-controller-using-td3.html
- **Key Contribution**: Describes Twin-Delayed DDPG (TD3) algorithm for learning PI gains directly from simulation/experiment without tuning rules.
- **Relevance**: Demonstrates practical implementation of learning-based PI autotuning in MATLAB environment (compatible with SPINOTOR workflow).

### 33. **Adaptive PI Controller Based on Reinforcement Learning for DC Motor Speed Control**
- **Source**: PMC/MDPI (2023)
- **URL**: https://www.mdpi.com/2313-7673/8/5/434
- **Key Contribution**: RL-based adaptive PI controller that learns gains online during operation. Shows improved speed response compared to fixed-gain PI.
- **Relevance**: Demonstrates real-time adaptive control capability that extends beyond offline auto-calibration.

### 34. **Machine Learning Techniques for Vector Control of PMSM Drives**
- **Source**: Taylor & Francis Online (2024)
- **URL**: https://www.tandfonline.com/doi/full/10.1080/23311916.2024.2323813
- **Key Contribution**: Survey of machine learning (neural networks, SVM, linear regression) for PMSM control. Covers speed control, torque ripple reduction, and parameter estimation.
- **Relevance**: Reviews ML-based approaches for multiple control objectives beyond just PI gain tuning.

### 35. **Model Predictive Control of Sensorless PMSM Using Unscented Kalman Filter - MDPI**
- **Source**: Energies Journal, MDPI (2024)
- **URL**: https://www.mdpi.com/1996-1073/17/10/2387
- **Key Contribution**: Combines MPC with UKF for sensorless PMSM control. Shows superior dynamic response compared to traditional PI-based control.
- **Relevance**: Provides alternative advanced control strategy with explicit dynamic optimization (useful for comparison with PI-based auto-tuning results).

---

## Sliding Mode Control and Advanced Observers

### 36. **Sliding-Mode Control Strategies for PMSM Speed Control: A Comprehensive Review, Taxonomy and Research Gaps**
- **Source**: arXiv Preprint (2024)
- **URL**: https://arxiv.org/html/2510.18420v1
- **Key Contribution**: Comprehensive survey of 200+ papers on SMC-based PMSM control (2020-2025). Classifies control approaches, surface designs, and disturbance-observer integration strategies.
- **Relevance**: Provides taxonomy of advanced control alternatives and identifies research gaps relevant to robust auto-calibration.

### 37. **Finite Control Set Model Predictive Control of PMSM Current Based on Super-Twisting SMO**
- **Source**: PLOS One (2024, Open Access)
- **URL**: https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0336702
- **Key Contribution**: Combines finite control set MPC with super-twisting SMO for enhanced current control. Eliminates traditional PI current loop.
- **Relevance**: Demonstrates modern alternative control approach with explicit discrete-time dynamics handling.

### 38. **An Improved Full-Speed Domain Sensorless Control Scheme Based on Hybrid Observer**
- **Source**: MDPI Electronics (2023)
- **URL**: https://www.mdpi.com/2079-9292/12/18/3759
- **Key Contribution**: Hybrid observer combining SMO and flux observer for full speed range sensorless control. Includes disturbance rejection optimization.
- **Relevance**: Shows observer architecture for seamless operation across startup to high-speed regions where PI tuning impacts overall performance.

---

## Design Guidelines and Performance Metrics

### 39. **A Review on Robust Predictive Control of PMSM Drives: Strategies and Future Trends**
- **Source**: AIP Advances (2024, Open Access)
- **URL**: https://pubs.aip.org/aip/adv/article/15/11/110701/3371458
- **Key Contribution**: Comprehensive review of robust predictive control strategies. Emphasizes three robustness approaches: disturbance observation, online parameter identification, and model improvement.
- **Relevance**: Provides framework for assessing robustness of auto-tuned gains under parameter uncertainty and load variations.

### 40. **Optimizing Sensorless Control in PMSM Based on SOGIFO-X Flux Observer - PMC**
- **Source**: PMC/NIH Journal (2023)
- **URL**: https://pmc.ncbi.nlm.nih.gov/articles/PMC10857136/
- **Key Contribution**: SOGIFO-X (Second-Order Generalized Integrator Flux Observer Extended) algorithm for robust sensorless control. Addresses DC bias and high-order harmonics in flux estimation.
- **Relevance**: Demonstrates advanced observer tuning methodology that interacts with cascaded PI control structure.

---

## Comparative Studies and Validation Methods

### 41. **A Comparative Experimental Analysis of PMSM Between Deadbeat Prediction Current Control and Field-Oriented Control**
- **Source**: ScienceDirect (2019)
- **URL**: https://www.sciencedirect.com/science/article/pii/S1876610219304023
- **Key Contribution**: Direct experimental comparison of PI-based FOC versus deadbeat predictive control. Validates dynamic response metrics and provides performance benchmarks.
- **Relevance**: Provides experimental validation methodology and performance metrics for comparing different auto-tuning approaches.

### 42. **Proportional-Type Robust Current Controller Under Variable Bandwidth Technique for PMSM - Scientific Reports**
- **Source**: Nature Scientific Reports (2024)
- **URL**: https://www.nature.com/articles/s41598-024-77701-2
- **Key Contribution**: Variable bandwidth current controller that adapts gain to operating condition. Shows improved robustness across speed range.
- **Relevance**: Demonstrates gain-scheduling approach that extends auto-tuning beyond single fixed bandwidth design.

### 43. **Enhanced Speed and Current Control of PMSM - DIVA Portal Thesis**
- **Source**: DIVA Portal (University Thesis)
- **URL**: https://www.diva-portal.org/smash/get/diva2:1248342/FULLTEXT01.pdf
- **Key Contribution**: Combines enhanced current control design with speed loop optimization. Includes experimental validation on laboratory PMSM testbed.
- **Relevance**: Practical thesis work demonstrating complete cascaded control design with experimental validation methodology.

---

## Practical Implementation References

### 44. **Stepper-FOC-MPC: Open Source Vector Control Implementation**
- **Source**: GitHub Repository
- **URL**: https://github.com/GinoAvanzini/Stepper-FOC-MPC
- **Key Contribution**: Open-source implementation of FOC with PI control, deadbeat predictive control, and FCS-MPC for two-phase PMSM (stepper motor). Provides reference code and Simulink models.
- **Relevance**: Demonstrates practical open-source implementation of multiple control approaches with available source code for reference.

### 45. **MCAF Documentation on FOC Implementation**
- **Source**: Microchip Motor Control Abstraction Framework
- **URL**: https://microchiptech.github.io/mcaf-doc/
- **Key Contribution**: Complete open-source motor control framework with FOC, parameter identification, and tuning modules. Production-grade reference implementation.
- **Relevance**: Industry-standard open-source reference for complete FOC system implementation including auto-tuning capabilities.

---

## Summary of Key Takeaways for Auto-Calibration Module

### Analytical Bandwidth-Based PI Tuning
1. **Current loop bandwidth** should be designed as **ωc = R/L** (natural frequency of L-R time constant)
2. **Speed loop bandwidth** typically targets **1/10th of current loop** to ensure cascade stability
3. **Phase margin** should be in 45-80° range, with 60° as conservative default
4. **Target PWM frequency ratios**: 5-15% of switching frequency (1-3 kHz typical)

### Anti-Windup and Cascade Robustness
1. Implement **back-calculation anti-windup** on both current and speed PI loops
2. Outer loop (speed) should see **saturated inner loop response** to prevent integrator windup
3. Use **tracking mode** to maintain continuous loop action during saturation
4. Validate **cascade stability** using structured singular value (μ) analysis

### Validation Criteria for Loaded Operation
1. **Speed tracking error** <5% during step changes
2. **Efficiency** maintained within 2% of nameplate (temperature-dependent)
3. **Flux angle orthogonality** (d-axis decoupling) <5% cross-coupling current
4. **Current ripple** <20% of rated current (system-dependent)
5. **Stability margins** across 0.5×rated to 1.5×rated load

### Online Parameter Identification Integration
1. Use **MRAS or RLS algorithm** for continuous motor parameter tracking
2. Adapt **Kp/Ki gains** to estimated **R/L ratio** for bandwidth maintenance
3. Account for **stator resistance** temperature coefficient (typically +0.4%/°C copper)
4. Validate **parameter convergence** before modifying controller gains

### Startup and Sensorless Transition
1. Use **I/f (constant current acceleration) mode** for reliable startup torque
2. Transition to **flux observer/SMO** at ~20-30% rated speed
3. Ensure **smooth current loop bandwidth** across startup transition
4. Validate **zero-speed operation** with dedicated test procedure

---

## References Organized by Application Context

### For Current Loop Design:
- References 1, 3, 4, 5, 6, 7 (bandwidth-based methods)
- References 33, 42 (adaptive bandwidth)
- References 26, 28 (advanced observer interaction)

### For Speed Loop and Cascaded Control:
- References 20, 21, 22, 23, 24, 25 (anti-windup and cascade design)
- References 39, 41 (robustness analysis)
- References 31, 32, 33 (learning-based adaptation)

### For Sensorless Operation:
- References 9, 10, 11, 12 (SMO and startup)
- References 26, 27, 28, 29, 30 (robustness and stability)
- References 40 (advanced observer algorithms)

### For Parameter Identification:
- References 13, 14, 15, 16 (identification algorithms)
- References 2, 3 (online parameter tracking)
- References 34 (ML-based identification)

### For Field Weakening:
- References 17, 18, 19 (field weakening control)
- References 29, 38 (operation in extended speed range)

### For Advanced Control Alternatives:
- References 8, 35, 36, 37 (MPC and SMC)
- References 31, 32, 33, 34 (learning-based control)

---

## Document Management

**Last Updated**: March 2026
**Total References**: 45+ papers and technical documents
**Focus Areas**: FOC auto-calibration, PI tuning, sensorless control, cascaded control robustness
**Target Motors**:
- Motenergy ME1718 (48V, 4000rpm, 14.3Nm)
- Motenergy ME1719 (48V, 4000rpm, 14.3Nm)
- Innotec 255-EZS48-160 (48V, 3000rpm, 26.5Nm)

This bibliography provides a comprehensive foundation for improving the FOC auto-calibration module in the SPINOTOR project, with particular emphasis on bandwidth-based PI tuning, anti-windup strategies, robustness validation, and integration with online parameter identification.
