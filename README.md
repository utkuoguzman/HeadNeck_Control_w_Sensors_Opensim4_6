**HEAD NECK SYSTEM WITH FEEDBACK-CONTROL STYLE (CMC) VOR/COR/VCR/CCR CONTROLLERS**

OSİM Model (MOSTLY MATURED)
* My interest is neck and eye muscles, so the repo also has an .osim which uses the combination of HYOID-Scaled model and the Upastras model.

Proprioception (WIP):
* So with lots of trial and error (with the help of Gemini), I made the original Millard12EdMuscleWithAfferents.cpp codes work in OpenSim 4.6. This plugin works with the new implicit solvers. It solves all muscles with the real time factor 1:40 (but there is a 33s burn-in time every time you start). For that to make it happen I had to change all the discontinuities with smoothing functions hence it is not exactly equal to Mileusnic's.
* But there is also Zheng's simplified model, when I want simple models I use that.
* The proprioception models are NOT compared to the literature yet. _WILL COME BACK TO THIS LATER 4!_

Vestibular (WIP):
* So with lots of trial and error (with the help of Gemini), I made replicated Zheng's vestibular system study as well. The axis are non orthogonal.
* It solves both inner ears with the real time factor 1:20 (but there is a 13s burn-in time every time you start)
* The vestibular model is NOT compared to the literature yet._ WILL COME BACK TO THIS LATE 3_!

Sensors Real-Time Factor:
* Real time factor of 1:60 (46 seconds burn in) on 13600K. Which is real handy considering 1.1M AdEx Neuron with 1K synapse of NEST-GPU can run around 45s on RTX 4070 Ti. 

Convergence Controller (MOSTLY MATURED):
* I need this just to mimic the human behavior of natural convergence with is around 1m (with great variations from people to people from 40cm to infinity).
* I takes a head frame target, and finds the necessary muscle commands to keep it. the VOR/COR controller will add onto it.
* So far it works good.

VOR/COR Controller (WIP):
* Their main mission is to use CMC to excert the exact opposite of the neck motion to both eyes so they (approximately) keep looking at the same point. Usually a true visual convergence-accomodation controller is required to keep the eyes on a target dead-on. The aim of this project is NOT the latter but the former.
* So far it works good.
* the "vestibular" and "cervico" parts are momentarily closed now._ WILL COME BACK TO THIS LATER2_!

VCR/CCR Controller (WIP):
* Their main mission is to use CMC to keep the neck in a desired attitude.
* So far it works good.
* the "vestibular" and "cervico" parts are momentarily closed now._ WILL COME BACK TO THIS LATER 1_!

Sensors + Controllers Real Time Factor:
* If you wish to use the native OpenSim controller scheme, then the final result is around 1:240.5 (with 3.5s burn-in, which is weird). This will be my initial case, but then I will begin using NEST-GPU (Go neural, people!).

LICENCE
* No nothing. Use anything to your liking. Share and inform me if you do cool stuffs with these.
