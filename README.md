* HEAD NECK SYSTEM WITH FEEDBACK-CONTROL STYLE (CMC) VOR/COR/VCR/CCR CONTROLLERS

OSİM Model (MOSTLY MATURED)
* My interest is neck and eye muscles, so the repo also has an .osim which uses the combination of HYOID-Scaled model and the Upastras model.

Proprioception (WIP):
* So with lots of trial and error (with the help of Gemini), I made the original Millard12EdMuscleWithAfferents.cpp codes work in OpenSim 4.6. This plugin works with the new implicit solvers. It solves all muscles with the real time factor 1:72. For that to make it happen I had to change all the discontinuities with smoothing functions hence it is not exactly equal to Mileusnic's.
* But there is also Zheng's simplified model, when I want simple models I use that.
* The proprioception models are NOT compared to the literature yet. WILL COME BACK TO THIS LATER!

Vestibular (WIP):
* So with lots of trial and error (with the help of Gemini), I made replicated Zheng's vestibular system study as well. The axis are non orthogonal.
* The vestibular model is NOT compared to the literature yet. WILL COME BACK TO THIS LATER!

Convergence Controller (WIP):
* I need this just to mimic the human behavior of natural convergence with is around 1m (with great variations from people to people from 40cm to infinity).
* I takes a head frame target, and finds the necessary muscle commands to keep it. the VOR/COR controller will add onto it.
* So far it works bad. WILL COME BACK TO THIS LATER 1!

VOR/COR Controller (WIP):
* Their main mission is to use CMC to excert the exact opposite of the neck motion to both eyes so they (approximately) keep looking at the same point. Usually a true visual convergence-accomodation controller is required to keep the eyes on a target dead-on. The aim of this project is NOT the latter but the former.
* So far it works bad. WILL COME BACK TO THIS LATER 2!
* the "vestibular" and "cervico" parts are momentarily closed now. WILL COME BACK TO THIS LATER 4!

VCR/CCR Controller (WIP):
* Their main mission is to use CMC to keep the neck in a desired attitude.
* So far it works good.
* the "vestibular" and "cervico" parts are momentarily closed now. WILL COME BACK TO THIS LATER 3!

LICENCE
* No nothing. Use anything to your liking. Share and inform me if you do cool stuffs with these.
