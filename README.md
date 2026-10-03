**HEAD NECK SYSTEM WITH FEEDBACK-CONTROL STYLE (CMC) VOR/COR/VCR/CCR CONTROLLERS**

OSİM Model (MOSTLY MATURED)
* My interest is neck, eye muscles and binaural hearing. So the repo also has an .osim which uses the combination of HYOID-Scaled model and the Upastras model.

Proprioception (MOSTLY MATURED)
* So with lots of trial and error (with the help of Gemini), I made the original Millard12EdMuscleWithAfferents.cpp codes work in OpenSim 4.6. This plugin works with the new implicit solvers. It solves all muscles with the real time factor less than 1:30. For that to make it happen I had to change all the discontinuities with smoothing functions hence it is not exactly equal to Mileusnic's.
* The proprioception models are compared to the literature.

Vestibular (MOSTLY MATURED)
* So with lots of trial and error (with the help of Gemini), I made replicated Zheng's vestibular system study as well. The axis are non orthogonal.
* It solves both inner ears with the real time factor less than 1:30.
* The vestibular model is NOT compared to the literature yet.
* When I use NEST, I will individually simulate hair cell output which will drive RT factor to 1:65.

Cochlea (NONE):
* So far there is none, the plan is to have offline calculated HRTFs and RIRs, and then convolve them in the iGPU or on the efficiency cores so that it keeps up with the simulation.
* Will probably not drive the simulation cost any further.

Sensors Real-Time Factor:
* Real time factor of proprioception and vestibular is ~1:30 on 13600K. Which is really handy considering 1.1M AdEx Neuron with 1K synapse of NEST-GPU can run around 45s on RTX 4070 Ti. 

Convergence Controller (MOSTLY MATURED)
* I need this just to mimic the human behavior of natural convergence with is around 1m (with great variations from people to people from 40cm to infinity).
* I takes a head frame target, and finds the necessary muscle commands to keep it. the VOR/COR controller will add onto it.
* So far it works good.

VCR/CCR Controller (GOING FINE):
* Their main mission is to use CMC to keep the neck in a desired attitude.
* So far it works good.
* the "vestibular" component is open to it.
* The "cervico" component is open to it.
* ONGOING >> Currently being compared to the literature!

VOR/COR Controller (VERY WIP):
* Their main mission is to use CMC to excert the exact opposite of the neck motion to both eyes so they (approximately) keep looking at the same point. Usually a true visual convergence-accomodation controller is required to keep the eyes on a target dead-on. The aim of this project is NOT the latter but the former.
* So far it works okay, but needs literature comparison._ WILL COME BACK TO THIS LATER1!_
* the "vestibular" part is momentarily closed now._ WILL COME BACK TO THIS LATER2!_
* the "cervico" parts is momentarily closed now._ WILL COME BACK TO THIS LATER2_!
* Momentarily I closed it until I solve VCR/CCR.

GVS (NONE):
* Currently none.
* Will probably drive the RT factor to 1:60.

Sensors + Controllers Real Time Factor:
* If you wish to use the native OpenSim controller scheme, then the final result is around 1:60. This will be my initial case, but then I will begin using NEST-GPU (Go neural, people!).

LICENCE
* No nothing. Use anything to your liking. Share and inform me if you do cool stuffs with these.
