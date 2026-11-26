from collections import defaultdict
import math
from scipy.optimize import brentq

def make_dict():
	return defaultdict(make_dict)

def Pi(T, Vvert):
	return 0.8554*T*(Vvert/2.0 + math.sqrt((Vvert/2.0)**2 + T/(0.3051)**2))

def Pp(T):
	return 0.3177*(T**1.5)

def Ppar(Vair):
	return 0.0296*(Vair**3)

def Thrust(m, Vair):
	return math.sqrt(((1.5 + m)*9.8 - 0.0279*(Vair*math.cos(10*math.pi/180.0))**2)**2 + (0.0296*Vair**2)**2)

def SpeedFunction(m, Vair, distance, energy_left):
	newT = Thrust(m, Vair)
	return (distance/float(Vair))*(Pi(newT, 0) + Pp(newT) + Ppar(Vair)) - (energy_left - 0.00001)

def give_endurance(node, vehicle, travel, v, i, j, k, totalTimeij, totalTimejk, speedij, speedjk):

	p 		= vehicle[v].takeoffSpeed
	qij 	= speedij
	qjk 	= speedjk
	r 		= vehicle[v].landingSpeed

	totalTimeij = travel[v][i][j].totalTime
	totalTimejk = travel[v][j][k].totalTime
	TTvij 	= travel[v][i][j].takeoffTime
	FTvij 	= totalTimeij - travel[v][i][j].takeoffTime - travel[v][i][j].landTime
	LTvij 	= travel[v][i][j].landTime

	TTvjk 	= travel[v][j][k].takeoffTime
	FTvjk 	= totalTimejk - travel[v][j][k].takeoffTime - travel[v][j][k].landTime
	LTvjk 	= travel[v][j][k].landTime
	sj_index = node[j].map_key
	sj = node[sj_index].serviceTimeUAV/60
	mj = (node[j].parcelWtLbs)*0.453592
	E = vehicle[v].batteryPower

	minimum_time_required = TTvij + FTvij + LTvij + sj + TTvjk + FTvjk + LTvjk
	Total_route_length = travel[v][i][j].totalDistance + travel[v][j][k].totalDistance
	if travel[v][i][j].totalDistance > 100 or travel[v][j][k].totalDistance > 100:
		return -1

	newT = Thrust(mj, 0)
	Ea = TTvij*(Pi(newT, p) + Pp(newT))

	newT = Thrust(mj, qij)
	Eb = FTvij*(Pi(newT, 0) + Pp(newT) + Ppar(qij))

	newT = Thrust(mj, 0)
	Ec = LTvij*(Pi(newT, r) + Pp(newT))

	newT = Thrust(0, 0)
	Ed = TTvjk*(Pi(newT, p) + Pp(newT))

	newT = Thrust(0, qjk)
	Ee = FTvjk*(Pi(newT, 0) + Pp(newT) + Ppar(qjk))

	newT = Thrust(0, 0)
	Ef = LTvjk*(Pi(newT, r) + Pp(newT))

	minimum_energy_required = Ea + Eb + Ec + Ed + Ee + Ef

	energy_left = E - minimum_energy_required

	if energy_left >= 0:
		newT = Thrust(0, 0)
		PHover = Pi(newT, 0) + Pp(newT)

		endurance = minimum_time_required + float(energy_left)/PHover

	else:
		endurance = -1

	return endurance

def modify_speed(oldLaunchTime, oldLandTime, newLaunchTime, newLandTime, v, i, j, k, node, vehicle, travel, totalTimeij, totalTimejk, speedij, speedjk):
	oldDuration = oldLandTime - oldLaunchTime - vehicle[v].recoveryTime
	newDuration = newLandTime - newLaunchTime - vehicle[v].recoveryTime

	p 		= vehicle[v].takeoffSpeed
	r 		= vehicle[v].landingSpeed
	TTvij 	= travel[v][i][j].takeoffTime
	FTvij 	= totalTimeij - travel[v][i][j].takeoffTime - travel[v][i][j].landTime
	LTvij 	= travel[v][i][j].landTime

	TTvjk 	= travel[v][j][k].takeoffTime
	FTvjk 	= totalTimejk - travel[v][j][k].takeoffTime - travel[v][j][k].landTime
	LTvjk 	= travel[v][j][k].landTime
	sj = node[j].serviceTimeUAV
	mj = (node[j].parcelWtLbs)*0.453592
	E = vehicle[v].batteryPower

	newT = Thrust(mj, 0)
	Ea = TTvij*(Pi(newT, p) + Pp(newT))

	newT = Thrust(mj, speedij)
	Eb = FTvij*(Pi(newT, 0) + Pp(newT) + Ppar(speedij))

	newT = Thrust(mj, 0)
	Ec = LTvij*(Pi(newT, r) + Pp(newT))

	newT = Thrust(0, 0)
	Ed = TTvjk*(Pi(newT, p) + Pp(newT))

	newT = Thrust(0, 0)
	Ef = LTvjk*(Pi(newT, r) + Pp(newT))

	energy_spent = Ea + Eb + Ec + Ed + Ef

	energy_left = E - energy_spent

	newFTvjk = newDuration - totalTimeij - sj - TTvjk - LTvjk
	if newFTvjk >= 0:
		newspeedjk = speedjk*FTvjk/float(newFTvjk)
		changeij = False
	else:
		newspeedjk = float(vehicle[v].cruiseSpeed)
		newFTvjk = speedjk*FTvjk/float(newspeedjk)
		changeij = True

	if newspeedjk > vehicle[v].cruiseSpeed:
		newspeedjk = float(vehicle[v].cruiseSpeed)
		newFTvjk = speedjk*FTvjk/float(newspeedjk)
		changeij = True

	newT = Thrust(0, newspeedjk)
	Ee = newFTvjk*(Pi(newT, 0) + Pp(newT) + Ppar(newspeedjk))

	if Ee > energy_left:

		if SpeedFunction(0, speedjk, speedjk*FTvjk, energy_left) < 0:
			newspeedjk = brentq(lambda actVar: SpeedFunction(0, actVar, speedjk*FTvjk, energy_left), speedjk, vehicle[v].cruiseSpeed)
		elif SpeedFunction(0, speedjk, speedjk*FTvjk, energy_left) < 0.00001:
			newspeedjk = float(speedjk)
		else:
			print("ERROR: Energy calculation wrong. Exiting...")
			exit()
		newFTvjk = speedjk*FTvjk/float(newspeedjk)
		newT = Thrust(0, newspeedjk)
		Ee = newFTvjk*(Pi(newT, 0) + Pp(newT) + Ppar(newspeedjk))
		changeij = False

	newFTvij = float(FTvij)
	newspeedij = float(speedij)

	if changeij == True:

		newFTvij = newDuration - TTvij - LTvij - sj - TTvjk - newFTvjk - LTvjk
		if newFTvij >= 0:
			newspeedij = speedij*FTvij/float(newFTvij)
		else:
			newspeedij = float(vehicle[v].cruiseSpeed)
			newFTvij = speedij*FTvij/float(newspeedij)

		if newspeedij > vehicle[v].cruiseSpeed:
			newspeedij = float(vehicle[v].cruiseSpeed)
			newFTvij = speedij*FTvij/float(newspeedij)

		newT = Thrust(mj, newspeedij)
		Eb = newFTvij*(Pi(newT, 0) + Pp(newT) + Ppar(newspeedij))

		energy_left = E - (Ea + Ec + Ed + Ee + Ef)

		if Eb > energy_left:

			if SpeedFunction(mj, speedij, speedij*FTvij, energy_left) < 0:
				newspeedij = brentq(lambda actVar: SpeedFunction(mj, actVar, speedij*FTvij, energy_left), speedij, vehicle[v].cruiseSpeed)
			elif SpeedFunction(mj, speedij, speedij*FTvij, energy_left) < 0.00001:
				newspeedij = float(speedij)
			else:
				print("ERROR: Energy calculation wrong. Exiting...")
				exit()
			newFTvij = speedij*FTvij/float(newspeedij)
			newT = Thrust(mj, newspeedij)
			Eb = newFTvij*(Pi(newT, 0) + Pp(newT) + Ppar(newspeedij))

	if Ea + Eb + Ec + Ed + Ee + Ef - E > 0:
		print("Energy calculation wrong. Exiting...")
		exit()

	if newFTvij <= 0:
		print("wrong i->j calculation. Exiting...")
		exit()

	if newFTvjk <= 0:
		print("wrong j->k calculation. Exiting...")
		exit()

	if (round(newspeedij,5) < round(speedij,5)) or (round(newspeedij,5) > round(vehicle[v].cruiseSpeed,5)):
		print("wrong i->j speed calculation. Exiting...")
		exit()

	if (round(newspeedjk,5) < round(speedjk,5)) or (round(newspeedjk,5) > round(vehicle[v].cruiseSpeed,5)):
		print("wrong j->k speed calculation. Exiting...")
		exit()

	checktprime_at_j = newLaunchTime + TTvij + newFTvij + LTvij
	hattprime_at_j = checktprime_at_j + sj
	checktprime_at_k = hattprime_at_j + TTvjk + newFTvjk + LTvjk + vehicle[v].recoveryTime

	newtotalTimeij = TTvij + newFTvij + LTvij
	newtotalTimejk = TTvjk + newFTvjk + LTvjk

	return [checktprime_at_j, hattprime_at_j, checktprime_at_k, newtotalTimeij, newtotalTimejk, newspeedij, newspeedjk]

def EndSpecs(node, vehicle, travel, v, i, j, k, distij, distjk, speedij, speedjk, idletime):

	p 		= vehicle[v].takeoffSpeed
	qij 	= speedij
	qjk 	= speedjk
	r 		= vehicle[v].landingSpeed
	TTvij 	= travel[v][i][j].takeoffTime
	FTvij 	= distij/float(speedij)
	LTvij 	= travel[v][i][j].landTime

	TTvjk 	= travel[v][j][k].takeoffTime
	FTvjk 	= distjk/float(speedjk)
	LTvjk 	= travel[v][j][k].landTime
	sj = node[j].serviceTimeUAV
	mj = (node[j].parcelWtLbs)*0.453592
	E = vehicle[v].batteryPower

	minimum_time_required = TTvij + FTvij + LTvij + sj + TTvjk + FTvjk + LTvjk

	newT = Thrust(mj, 0)
	Ea = TTvij*(Pi(newT, p) + Pp(newT))

	newT = Thrust(mj, qij)
	Eb = FTvij*(Pi(newT, 0) + Pp(newT) + Ppar(qij))

	newT = Thrust(mj, 0)
	Ec = LTvij*(Pi(newT, r) + Pp(newT))

	newT = Thrust(0, 0)
	Ed = TTvjk*(Pi(newT, p) + Pp(newT))

	newT = Thrust(0, qjk)
	Ee = FTvjk*(Pi(newT, 0) + Pp(newT) + Ppar(qjk))

	newT = Thrust(0, 0)
	Ef = LTvjk*(Pi(newT, r) + Pp(newT))

	minimum_energy_required = Ea + Eb + Ec + Ed + Ee + Ef

	energy_left = E - minimum_energy_required

	if energy_left >= 0:
		newT = Thrust(0, 0)
		PHover = Pi(newT, 0) + Pp(newT)

		endurance = minimum_time_required + float(energy_left)/PHover

	else:
		endurance = -1

	enduranceUsed = minimum_time_required + idletime

	energyUsed = minimum_energy_required + PHover*idletime

	return [endurance, minimum_time_required, enduranceUsed, minimum_energy_required, energyUsed]
