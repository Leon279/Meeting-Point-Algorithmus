import os
import sys
import traci
from dataclasses import dataclass
import string
import random
import sumolib
import time
import copy


@dataclass
class Package:
    target: str
    name: str

    def __repr__(self):
        return self.name


vehicles = {}
states = {}
simstart = "7000"


def step():
    s = traci.simulation.getTime()
    traci.simulationStep()  # 1 step is 1 second
    return s


def run_sim(name):
    global lkw, a, p, alpha
    a = 4               # Anzahl an zu initialisierenden Trucks
    p = 12              # Anzahl an packages die generiert werden sollen
    lkw = "LKW"         # Name des vehicles, das initial alle Pakete hat (darf nicht "Truck_" + bel. Zahl > 0 sein)
    scenario = 1        # welches Szenario ausgeführt werden soll
    alpha = float(0.5)  # Wert alpha für Berechnung im Meeting-Point Algorithmus
    random.seed(44)     # seed für s, t und U
    pseed = 4        # random.seed für packages
    doAlgorithm = True     # ob der Meeting-Point Algorithmus ausgeführt werden soll
    randompoints = True     # ob alle Punkte zufällig generiert werden sollen
    if randompoints:
        U, s, t, U2 = initialize(pseed)   # alle Punkte werden zufällig generiert (wird in Evaluation verwendet)
    else:
        U, s, t = initializeSetPoints(pseed)            # innerhalb der Methode initialsetpoints können s, t und U manuell festgelegt werden. U2 ist hier nicht nötig, da Trucks erst nach Sortierung von U erstellt werden, somit bleibt die Reihenfolge bestehen
        U2 = tuple(U)
    print("Es sind s, t, U: " + str(s) + ", " + str(t) + ", " + str(U))
    print("mit U2: " + str(U2))
    if doAlgorithm:
        start_time = time.time()
        cost, route, meetingpoints = algorithm1paper(U, s, t)
        end_time = time.time()
        time_lapsed = end_time - start_time
        time_convert(time_lapsed)
        print("cost:")
        print(cost)
        print("route:")
        print(route)
        print("meetingpoints:")
        for key in meetingpoints:
            print("_______")
            print(str(key) + ":")
            print(meetingpoints[key])
        input("Press Enter to continue...")
    if not doAlgorithm and scenario == 1:
        # manuelle Eingabe der Route und Meeting-Points für Simulation mit Paketauslieferung, wenn diese vorher mit Meeting-Point Algorithmus berechnet wurden
        route = ['-2836', '-9068', '-10808', '-18974', '-31622#10-AddedOnRampNode', '-31622#10-AddedOffRampNode', '-12400', '-29484', '-17304', '-8706', '-16448', '-29288', '-11848', '-27536', '-1740', '-26640', '-16014', '-24224', '-7900', '-17878', '-5428', '-17274', '-18074', '-5178', '-7224', '-1466', '-20604', '-9490', '-2052', '-4948', '-19544', '-736', '-19346', '-28752', '-29988', '-18432', '-18886', '-20080', '-17084', '-6086', '-28352', '-11722', '-17684', '-3976', '-7540', '-13350', '-25688', '-21152', '-17046', '-29900', '-10004', '-22042', '-22748', '-19212', '-2116', '-16788', '-17072', '-18708', '-18106', '-17732', '-23424', '-8762', '-28954', '-26432', '-13472', '-10042', '-7626']
        # Format in meetingpoints ist "andersherum" als das vom Meeting-Point erstellte meetingpoints dictionary: meetingpoints[Truck Startknoten aus U2] = Übergabepunkt, statt meetingpoints[Übergabepunkt] = Truck Startknoten aus U2,
        # da mehr als ein Truck denselben Übergabepunkt besitzen kann (derselbe key im dictionary)
        # entsprechend lässt sich Szenario 1 nicht unmittelbar nach der Ausführung des Meeting-Point Algorithmus ausführen, es müssen route und meetingpoints manuell eingegeben werden
        meetingpoints = {}
        meetingpoints['-28800'] = '-21152'
        meetingpoints['-1720'] = '-19346'
        meetingpoints['-2134'] = '-5428'
        meetingpoints['-1466'] = '-20604'
    net = sumolib.net.readNet('C:/Users/LSchu/PycharmProjects/SUMO/scenario/lust.net.xml')
    if scenario == 1:       # Konstruiere Route als edges Liste durch die des Meeting-Point Algorithmus berechnete Route mit junctions Liste
        edges = net.getEdges()
        alledges = []
        for element in edges:
            alledges.append(element.getID())
        lkwedgesroute = []
        for junc in range(len(route)-1):
            outgoing = net.getNode(route[junc]).getOutgoing()
            incoming = net.getNode(route[junc+1]).getIncoming()
            edger = list(set(outgoing).intersection(incoming))
            if len(edger) != 1:  # edge zwischen zwei Knoten muss eindeutig sein, damit die korrekte edges-Route konstruiert werden kann
                print("Es wurde evtl. eine falsche edge genommen, da zwei Knoten durch mehr als eine edge verbunden waren")
                input("Press Enter to continue...")
            for edge1 in outgoing:
                for edge2 in incoming:
                    if edge1.getID() == edge2.getID():
                        lkwedgesroute.append(edge1.getID())
    if scenario == 1:                   # LKW fährt berechnete Route des Meeting-Point Algorithmus und übergibt den kleineren Lieferfahrzeugen (Trucks) an den berechneten Meeting-Points die Pakete
        print("Szenario 1")
        pp = 0
        visited = [False] * a           # visited[i] sagt aus, ob der berechnete Meeting-Point meetingpoints[U2[i]] bereits vom LKW besucht wurde. Truck_(i+1) muss zu meetingpoints[U2[i]] fahren (nach Berechnung durch Meeting-Point Algorithmus)
        changedTargets = [False] * a    # changedTarget[i] sagt aus, ob das Ziel, wo Truck_i+1 hinfahren soll (Meeting-Point) bereits festgelegt wurde
        nextTarget = ['empty'] * a      # nextTarget[i] gibt die Edge an, zu der Truck_i+1 gerade hinfährt
        trucksfinished = [False] * a    # trucksfinished[i] sagt aus, ob Truck_i+1 bereits alle Pakete ausgeliefert hat
        truckdistances = [0] * a        # Truck_i+1 hat eine Distanz von truckdistances[i] zurückgelegt
        trucksteps = [0] * a            # Truck_i+1 hat trucksteps[i] Steps benötigt
        drivenRoute = []                # Die Route, die der LKW tatsächlich gefahren ist (um zu überprüfen ob es mit der Berechnung des Meeting-Point Algorithmus übereinstimmt
        lkwstopedge = net.getNode(t).getOutgoing()[0].getID()
        traci.vehicle.changeTarget(lkw, lkwedgesroute[pp])
        lkwfinished = False
        lastEdge = False
        for j in range(0, 50000):  # jump in the middle of the simulation
            time_stamp = step()
            for veh_id in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(veh_id) == "trailer":
                    if veh_id == lkw and not lkwfinished:
                        roadid = traci.vehicle.getRoadID(veh_id)
                        if roadid not in drivenRoute and roadid in alledges:
                            drivenRoute.append(roadid)
                            if pp < len(lkwedgesroute)-1:
                                if roadid == lkwedgesroute[pp]:
                                    traci.vehicle.changeTarget(veh_id, lkwedgesroute[pp+1])
                                    pp = pp + 1
                            if roadid == lkwedgesroute[len(lkwedgesroute)-1] and not lastEdge:
                                traci.vehicle.changeTarget(veh_id, lkwstopedge)
                                traci.vehicle.setStop(lkw, lkwstopedge, pos=traci.lane.getLength(str(lkwstopedge)+"_0")/2, duration=99999.0)  # in Mitte der straße stoppen, um einen evtl auftretenen und nicht aufzulösenden Stau zu verhindern
                                lastEdge = True
                            if roadid == lkwstopedge:
                                print("LKW ist am Ziel angekommen")
                                print("Vom Algorithmus berechnete Route:")
                                print(lkwedgesroute)
                                print("Gefahrene Route:")
                                print(drivenRoute)
                                lkwdistance = traci.vehicle.getDistance(veh_id)
                                lkwsteps = j
                                print("LKW Distance: " + str(lkwdistance) + ", Steps: " + str(lkwsteps))
                                lkwfinished = True
                        for key in meetingpoints:
                            outgoing = net.getNode(meetingpoints[key]).getOutgoing()
                            for outedge in outgoing:
                                if roadid == outedge.getID():
                                    index = U2.index(key)
                                    visited[index] = True
                                    print("Changed visited: " + str(visited))
                    for tr in range(a):
                        if veh_id == "Truck_" + str(tr+1):
                            if not changedTargets[tr]:
                                truckmeetingnode = meetingpoints[U2[tr]]                        # der Meeting-Point, zu dem Truck_(tr+1) fahren soll (nach Berechnung des Meeting-Point Algorithmus)
                                meetingedgelist = net.getNode(truckmeetingnode).getOutgoing()   # sicherstellen, dass ein Truck auf einer Edge stoppt, die sich auf der Route des LKWs befindet, da es sich hierbei eher um eine Hauptstraße handelt
                                for element in meetingedgelist:
                                    if element.getID() in lkwedgesroute:
                                        meetingedge = element.getID()
                                traci.vehicle.changeTarget(veh_id, meetingedge)
                                changedTargets[tr] = True
                                print(str(veh_id) + " fährt nun zu " + str(meetingedge) + " des Meetingpoint " + str(truckmeetingnode) + ", gestartet bei " + str(U2[tr]))
                                traci.vehicle.setStop(veh_id, meetingedge, pos=traci.lane.getLength(str(meetingedge)+"_0")/2, duration=99999.0)   # warte auf Ankunft des LKWs, evtl pos ändern, falls der Truck so ungünstig steht dass der LKW die Edge nicht mehr erreichen kann (erhöhen) oder die edge nicht lang genug ist um an der position zu stoppen (erniedrigen)
                            if veh_id == "Truck_1":            # Laufvariable tr bleibt hier stets auf 0, wegen "if veh_id == "Truck_" + str(tr+1):"
                                if visited[tr] and traci.vehicle.getStopState(veh_id) == 1:      # Wenn der Truck den Meeting-Point erreicht hat (nur dann ist stopstate = 1) und der LKW bereits dort gewesen ist, dürfen Pakete angenommen werden
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:           # Wenn der Truck das Ziel vom aktuell auszuliefernde Paket erreicht hat
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_2":            # Laufvariable tr bleibt hier stets auf 1, wegen "if veh_id == "Truck_" + str(tr+1):"
                                if visited[tr] and traci.vehicle.getStopState(veh_id) == 1:      # Wenn der Truck den Meeting-Point erreicht hat (nur dann ist stopstate = 1) und der LKW bereits dort gewesen ist, dürfen Pakete angenommen werden
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:           # Wenn der Truck das Ziel vom aktuell auszuliefernde Paket erreicht hat
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_3":           # Laufvariable tr bleibt hier stets auf 2, wegen "if veh_id == "Truck_" + str(tr+1):"
                                if visited[tr] and traci.vehicle.getStopState(veh_id) == 1:      # Wenn der Truck den Meeting-Point erreicht hat (nur dann ist stopstate = 1) und der LKW bereits dort gewesen ist, dürfen Pakete angenommen werden
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:           # Wenn der Truck das Ziel vom aktuell auszuliefernde Paket erreicht hat
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_4":          # Laufvariable tr bleibt hier stets auf 3, wegen "if veh_id == "Truck_" + str(tr+1):"
                                if visited[tr] and traci.vehicle.getStopState(veh_id) == 1:      # Wenn der Truck den Meeting-Point erreicht hat (nur dann ist stopstate = 1) und der LKW bereits dort gewesen ist, dürfen Pakete angenommen werden
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:           # Wenn der Truck das Ziel vom aktuell auszuliefernde Paket erreicht hat
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
            print(vehicles)
            # überprüfe, ob jeder Truck jedes Paket ausgeliefert hat
            emptydict = True
            for tr in range(len(trucksfinished)):
                if not trucksfinished[tr]:
                    emptydict = False
            if emptydict and lkwfinished:
                print("Alle Pakete wurden ausgeliefert")
                print("LKW Distance: " + str(lkwdistance) + ", Steps: " + str(lkwsteps))
                for tr in range(a):
                    print("Truck_" + str(tr+1) + " Distance: " + str(truckdistances[tr]) + ", Steps: " + str(trucksteps[tr]))
                print("Für LKW:")
                print("Vom Algorithmus berechnete Route:")
                print(lkwedgesroute)
                print("Gefahrene Route:")
                print(drivenRoute)
                input("Press Enter to continue...")
    if scenario == 2:   # '--30528#7' wird als treffpunkt in der stadtmitte festgelegt, wo alle Fahrzeuge hinfahren, damit der LKW die Pakete an die Trucks austeilen kann
        print("Szenario 2")
        # incoming edge von t generieren, die von '--30528#7' am nähesten an t ist, zu der der LKW als ziel fahren soll (wird als tedge gespeichert)
        incoming = net.getNode(t).getIncoming()
        tmin = traci.simulation.findRoute('--30528#7', incoming[0].getID()).travelTime
        tedge = incoming[0].getID()
        for edge in incoming:
            findroute = traci.simulation.findRoute('--30528#7', edge.getID())
            if findroute.travelTime < tmin:
                tmin = findroute.travelTime
                tedge = edge.getID()
        lkwfinished = False
        lkwchangedtarget = False
        visited = False                 # sagt aus, ob der LKW an '--30528#7' angekommen ist und die Pakete von den Trucks abgeholt werden können
        changedTargets = [False] * a    # changedTarget[i] sagt aus, ob für Truck_i+1 bereits festgelegt wurde, dass dieser zu '--30528#5' fahren soll
        nextTarget = ['empty'] * a      # nextTarget[i] gibt die Edge an, zu der Truck_i+1 gerade hinfährt
        trucksfinished = [False] * a    # trucksfinished[i] sagt aus, ob Truck_i+1 bereits alle Pakete ausgeliefert hat
        truckdistances = [0] * a        # Truck_i+1 hat eine Distanz von truckdistances[i] zurückgelegt
        trucksteps = [0] * a            # Truck_i+1 hat trucksteps[i] Steps benötigt
        for j in range(0, 50000):
            time_stamp = step()
            for veh_id in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(veh_id) == "trailer":
                    if veh_id == lkw and not lkwfinished:
                        if not lkwchangedtarget:
                            traci.vehicle.changeTarget(veh_id, '--30528#7')
                            lkwchangedtarget = True
                        if traci.vehicle.getRoadID(veh_id) == '--30528#7':
                            traci.vehicle.changeTarget(veh_id, tedge)
                            visited = True
                        if traci.vehicle.getRoadID(veh_id) == tedge:
                            outgoing = net.getNode(t).getOutgoing()
                            traci.vehicle.changeTarget(veh_id, outgoing[0].getID())  # noch zur junction t fahren, indem zu einer bel. ausgehenden Kante von t gefahren wird
                            traci.vehicle.setStop(veh_id, outgoing[0].getID(), pos=traci.lane.getLength(str(outgoing[0].getID())+"_0")/2, duration=99999.0)
                        if traci.vehicle.getStopState(veh_id) == 1:
                            print("LKW ist am Ziel angekommen")
                            lkwdistance = traci.vehicle.getDistance(veh_id)
                            lkwsteps = j + 2
                            print("LKW Distance: " + str(lkwdistance) + ", Steps: " + str(lkwsteps))
                            lkwfinished = True
                    for tr in range(a):
                        if veh_id == "Truck_" + str(tr+1):
                            if not changedTargets[tr]:
                                traci.vehicle.changeTarget(veh_id, '--30528#7')
                                changedTargets[tr] = True
                                # pos=length/(tr+1), da in diesem szenario alle trucks an derselben stelle stoppen wollen, sodass nur der erste tatsächlich stoppt und die restlichen bremsen und darauf warten, dass der platz frei wird, damit diese stoppen können
                                # Wenn ein vehicle zu lange bremst und es nicht weiter geht scheint es nach einer gewissen Zeit zu despawnen
                                traci.vehicle.setStop(veh_id, '--30528#7', pos=traci.lane.getLength("--30528#7_0")/(tr+1), duration=99999.0)
                            if veh_id == "Truck_1":
                                if traci.vehicle.getStopState(veh_id) == 1 and visited:
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_2":
                                if traci.vehicle.getStopState(veh_id) == 1 and visited:
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_3":
                                if traci.vehicle.getStopState(veh_id) == 1 and visited:
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_4":
                                if traci.vehicle.getStopState(veh_id) == 1 and visited:
                                    getPackages(net, veh_id)
                                    traci.vehicle.resume(veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:       # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
            print(vehicles)
            # überprüfe, ob jeder Truck jedes Paket ausgeliefert hat
            emptydict = True
            for tr in range(len(trucksfinished)):
                if not trucksfinished[tr]:
                    emptydict = False
            if emptydict and lkwfinished:
                print("Alle Pakete wurden ausgeliefert")
                print("LKW Distance: " + str(lkwdistance) + ", Steps: " + str(lkwsteps))
                for tr in range(a):
                    print("Truck_" + str(tr+1) + " Distance: " + str(truckdistances[tr]) + ", Steps: " + str(trucksteps[tr]))
                input("Press Enter to continue...")
    if scenario == 3:  # Es gibt keinen LKW, alle Lieferfahrzeuge fahren zu s um sich die Pakete abzuholen und liefern diese aus
        # Es wird weiterhin vehicles["LKW"] genutzt, um die Pakete zu verwalten. In diesem Szenario kann man sich vorstellen, dass der LKW an Knoten s steht und sich nicht fortbewegt
        print("Szenario 3")
        traci.vehicle.remove(lkw, 3)
        # beliebige outgoing edge von s generieren, zu der alle Lieferfahrzeuge fahren sollen (wird als sedge gespeichert)
        outgoing = net.getNode(s).getOutgoing()
        sedge = outgoing[0].getID()
        print("sedge: " + str(sedge))
        changedTargets = [False] * a    # changedTarget[i] sagt aus, ob für Truck_i+1 bereits festgelegt wurde, dass dieser zu s fahren soll
        nextTarget = ['empty'] * a      # nextTarget[i] gibt die Edge an, zu der Truck_i+1 gerade hinfährt
        trucksfinished = [False] * a    # trucksfinished[i] sagt aus, ob Truck_i+1 bereits alle Pakete ausgeliefert hat
        truckdistances = [0] * a        # Truck_i+1 hat eine Distanz von truckdistances[i] zurückgelegt
        trucksteps = [0] * a            # Truck_i+1 hat trucksteps[i] Steps benötigt
        for j in range(0, 50000):
            time_stamp = step()
            for veh_id in traci.vehicle.getIDList():
                if traci.vehicle.getTypeID(veh_id) == "trailer":
                    for tr in range(a):
                        if veh_id == "Truck_" + str(tr+1):
                            if not changedTargets[tr]:
                                traci.vehicle.changeTarget(veh_id, sedge)
                                changedTargets[tr] = True
                            if veh_id == "Truck_1":
                                if traci.vehicle.getRoadID(veh_id) == sedge:
                                    getPackages(net, veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_2":
                                if traci.vehicle.getRoadID(veh_id) == sedge:
                                    getPackages(net, veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_3":
                                if traci.vehicle.getRoadID(veh_id) == sedge:
                                    getPackages(net, veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
                            if veh_id == "Truck_4":
                                if traci.vehicle.getRoadID(veh_id) == sedge:
                                    getPackages(net, veh_id)
                                    nextTarget[tr] = doReroute(veh_id, net)
                                if traci.vehicle.getRoadID(veh_id) == nextTarget[tr]:
                                    if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                                        for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                            if package.target == nextTarget[tr]:
                                                indexpackage = vehicles[veh_id].index(package)
                                        print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                                        del vehicles[veh_id][indexpackage]
                                        if len(vehicles[veh_id]) != 0:
                                            nextTarget[tr] = doReroute(veh_id, net)
                                        else:   # um 1 eingerückt alles mit diesem else
                                            if not trucksfinished[tr]:
                                                print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                                truckdistance = traci.vehicle.getDistance(veh_id)
                                                truckdistances[tr] = truckdistance
                                                trucksteps[tr] = j
                                                trucksfinished[tr] = True
                                                print(str(veh_id) + " Distance: " + str(truckdistance) + ", Steps: " + str(trucksteps[tr]))
            print(vehicles)
            # überprüfe, ob jeder Truck jedes Paket ausgeliefert hat
            emptydict = True
            for tr in range(len(trucksfinished)):
                if not trucksfinished[tr]:
                    emptydict = False
            if emptydict:
                print("Alle Pakete wurden ausgeliefert")
                for tr in range(a):
                    print("Truck_" + str(tr+1) + " Distance: " + str(truckdistances[tr]) + ", Steps: " + str(trucksteps[tr]))
                input("Press Enter to continue...")
    if scenario == 4:  # Nur der LKW liefert alle Pakete aus, es gibt keine weiteren Lieferfahrzeuge
        print("Szenario 4")
        for tr in range(a):
            traci.vehicle.remove("Truck_" + str(tr+1), 3)
        nextTarget = 'empty'
        firstpackage = False    # firstpackage gibt an, ob der Route zu dem zuerst auszuliefernden Paket festgelegt wurde
        lkwfinished = False     # lkwfinished gibt an, ob LKW alle Pakete ausgeliefert hat
        for j in range(0, 50000):
            time_stamp = step()
            for veh_id in traci.vehicle.getIDList():
                if veh_id == lkw:
                    if not firstpackage:
                        nextTarget = doReroute(veh_id, net)
                        firstpackage = True
                    if traci.vehicle.getRoadID(veh_id) == nextTarget:
                        if len(vehicles[veh_id]) != 0:                              # Wenn noch Pakete übrig sind
                            for package in vehicles[veh_id]:                        # Ausgeliefertes Paket löschen
                                if package.target == nextTarget:
                                    indexpackage = vehicles[veh_id].index(package)
                            print(str(veh_id) + " hat " + str(vehicles[veh_id][indexpackage]) + " mit Target " + str(vehicles[veh_id][indexpackage].target) + " zugestellt")
                            del vehicles[veh_id][indexpackage]
                            if len(vehicles[veh_id]) != 0:
                                nextTarget = doReroute(veh_id, net)
                            else:   # um 1 eingerückt alles mit diesem else
                                if not lkwfinished:
                                    print(str(veh_id) + " hat alle Pakete ausgeliefert")
                                    lkwdistance = traci.vehicle.getDistance(veh_id)
                                    lkwsteps = j
                                    lkwfinished = True
                                    print(str(veh_id) + " Distance: " + str(lkwdistance) + ", Steps: " + str(lkwsteps))
                                    input("Press Enter to continue...")
            print(vehicles)


def doReroute(veh_id, net):
    minDistance = 9999999
    truckpos = traci.vehicle.getPosition(veh_id)
    for package in vehicles[veh_id]:
        FromNode = net.getEdge(package.target).getFromNode().getID()
        FromNodePos = traci.junction.getPosition(FromNode, False)
        distance = traci.simulation.getDistance2D(truckpos[0], truckpos[1], FromNodePos[0], FromNodePos[1], False, False)
        if distance < minDistance:
            minDistance = distance
            nextEdge = package.target
    print(str(veh_id) + " fährt nun zu " + str(nextEdge) + " mit Distanz " + str(minDistance))
    traci.vehicle.changeTarget(veh_id, nextEdge)
    return nextEdge


# Je nachdem, wie viele Trucks in der Simulation sind, werden die Bereiche anders unterteilt, zu denen die einzelnen Trucks die Pakete ausliefern müssen
# Position der Eingrenzung ist nicht genau bei 1/2, 1/3, oder 1/4 der maximalen Koordinatenzahl des Netzes, Bereiche wurden grob nach Anzahl der Kanten in diesem unterteilt
def getPackages(net, veh_id):
    if veh_id == "Truck_1":
        if a == 2:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] < 6620:               # alle Pakete auf der linken Hälfte der Karte soll Truck_1 übernehmen (bei 2 Trucks)
                    vehicles['Truck_1'].append(package)
            for package in vehicles['Truck_1']:         # alle von Truck_1 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        elif a == 3:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] < 4930:               # alle Pakete im linken Drittel der Karte soll Truck_1 übernehmen (bei 3 Trucks)
                    vehicles['Truck_1'].append(package)
            for package in vehicles['Truck_1']:         # alle von Truck_1 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        elif a == 4:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] < 6620 and FromNodePos[1] >= 6220:     # alle Pakete im linken, oberen Viertel der Karte soll Truck_1 übernehmen (bei 4 Trucks)
                    vehicles['Truck_1'].append(package)
            for package in vehicles['Truck_1']:         # alle von Truck_1 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        else:
            print("Anzahl von " + str(a) + " Trucks wird nicht unterstützt")
    if veh_id == "Truck_2":
        if a == 2:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] >= 6620:              # alle Pakete auf der rechten Hälfte der Karte soll Truck_2 übernehmen (bei 2 Trucks)
                    vehicles['Truck_2'].append(package)
            for package in vehicles['Truck_2']:        # alle von Truck_2 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        elif a == 3:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if 4930 <= FromNodePos[0] < 8000:               # alle Pakete im mittleren Drittel der Karte soll Truck_2 übernehmen (bei 3 Trucks)
                    vehicles['Truck_2'].append(package)
            for package in vehicles['Truck_2']:         # alle von Truck_2 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        elif a == 4:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] >= 6620 and FromNodePos[1] >= 6220:     # alle Pakete im rechten, oberen Viertel der Karte soll Truck_2 übernehmen (bei 4 Trucks)
                    vehicles['Truck_2'].append(package)
            for package in vehicles['Truck_2']:           # alle von Truck_2 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        else:
            print("Anzahl von " + str(a) + " Trucks wird nicht unterstützt")
    if veh_id == "Truck_3":
        if a == 3:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] >= 8000:               # alle Pakete im rechten Drittel der Karte soll Truck_3 übernehmen (bei 3 Trucks)
                    vehicles['Truck_3'].append(package)
            for package in vehicles['Truck_3']:         # alle von Truck_3 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        elif a == 4:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] < 6620 and FromNodePos[1] < 6220:       # alle Pakete im linken, unteren Viertel der Karte soll Truck_3 übernehmen (bei 4 Trucks)
                    vehicles['Truck_3'].append(package)
            for package in vehicles['Truck_3']:          # alle von Truck_3 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        else:
            print("Anzahl von " + str(a) + " Trucks wird nicht unterstützt")
    if veh_id == "Truck_4":
        if a == 4:
            for package in vehicles[lkw]:
                FromNode = net.getEdge(package.target).getFromNode().getID()
                FromNodePos = traci.junction.getPosition(FromNode, False)
                if FromNodePos[0] >= 6620 and FromNodePos[1] < 6220:     # alle Pakete im rechten, unteren Viertel der Karte soll Truck_4 übernehmen (bei 4 Trucks)
                    vehicles['Truck_4'].append(package)
            for package in vehicles['Truck_4']:        # alle von Truck_4 übernommenen Pakete in vehicles[LKW] löschen
                if package in vehicles[lkw]:
                    indexpackage = vehicles[lkw].index(package)
                    del vehicles[lkw][indexpackage]
        else:
            print("Anzahl von " + str(a) + " Trucks wird nicht unterstützt")


def algorithm1paper(U, s, t):
    net = sumolib.net.readNet('C:/Users/LSchu/PycharmProjects/SUMO/scenario/lust.net.xml')
    # Liste aller Knoten des Straßennetzes
    junctions = []
    edges = net.getEdges()
    for edge in edges:
        if edge.getFromNode().getID() not in junctions:
            junctions.append(edge.getFromNode().getID())
        if edge.getToNode().getID() not in junctions:
            junctions.append(edge.getToNode().getID())
    # add dummy node s2
    junctions.append('s2')
    w, h = len(junctions), len(junctions)
    # Matrix mit Abständen von jedem Knoten zu jedem benachbartem Knoten
    Matrix = [[-1 for x in range(w)] for y in range(h)]
    for k in range(len(junctions)):
        Matrix[k][k] = 0
    for FromNode in junctions:
        if FromNode != 's2':
            outgoing = net.getNode(FromNode).getOutgoing()
            for outedge in outgoing:
                ToNode = net.getEdge(outedge.getID()).getToNode().getID()
                i1 = junctions.index(FromNode)
                i2 = junctions.index(ToNode)
                weight = traci.edge.getTraveltime(outedge.getID())
                if Matrix[i1][i2] == -1 or Matrix[i1][i2] > weight:
                    Matrix[i1][i2] = weight
    s2index = junctions.index('s2')
    sindex = junctions.index(s)
    Matrix[s2index][sindex] = 0
    Matrix3 = Matrix        # Nur Distanzen von jedem Knoten aus Menge U zu jedem anderen Knoten
    for z in junctions:
        for u in junctions:
            if z in U and u != 's2':
                lowestcost = 999999
                outgoing = net.getNode(z).getOutgoing()
                incoming = net.getNode(u).getIncoming()
                for outedge in outgoing:  # Shortest Path von jeder ausgehenden Edge von z zu jeder eingehenden Kante zu u berechnen, am Ende den kürzesten nehmen
                    for inedge in incoming:
                        shortestpath = net.getShortestPath(outedge, inedge)
                        travelcost = 0
                        if shortestpath[0] is not None:
                            for edge in shortestpath[0]:
                                traveltime = traci.edge.getTraveltime(edge.getID())
                                travelcost = travelcost + traveltime
                            if travelcost < lowestcost:
                                lowestcost = travelcost
                iz = junctions.index(z)
                iu = junctions.index(u)
                Matrix3[iz][iu] = lowestcost
    # Beginn des Algorithmus
    Q = []
    D = []
    Q.append((0, ('s2', [])))
    while not len(Q) == 0:
        Q.sort()
        (cost, (v, X)) = Q[0]
        Q.remove(Q[0])
        X.sort()
        previousstate = (v, X)
        if v == t and X == U:
            route, meetingpoints = getRoute(t, U)
            return cost, route, meetingpoints
        if (v, X) not in D:
            D.append((v, X))
        iv = junctions.index(v)
        for iu in range(len(junctions)):
            if (v == 's2' and junctions[iu] == s) or Matrix[iv][iu] > 0:
                X1 = generateX1(U, X)
                for subset in X1:
                    summe = 0
                    for z in subset:
                        iz = junctions.index(z)
                        summe = summe + Matrix3[iz][iu]
                    cost2 = cost + (alpha * Matrix[iv][iu]) + ((1 - alpha) * summe)
                    X2 = generateX2(X, subset)
                    u = junctions[iu]
                    updatepaper(Q, D, u, X2, cost2, previousstate)
    return 999999999, ['invalid']


def updatepaper(Q, D, v, X, cost, previousstate):
    if (v, X) in D:
        return Q, D, v, X, cost, previousstate
    vXinQ = False
    for element in Q:
        if (v, X) in element:
            vXinQ = True
    if not vXinQ:
        Q.append((cost, (v, X)))
        states[(v, tuple(X))] = [previousstate]
    for element in Q:
        (Qcost, (Qv, QX)) = element
        if (Qv, QX) == (v, X):
            if cost < Qcost:
                Q.remove(element)
                Q.append((cost, (v, X)))
                states[(v, tuple(X))] = [previousstate]
    return Q, D, v, X, cost, previousstate


def getRoute(t, U):
    meetingpoints = {}
    route = [t]
    (v, X) = (t, U)
    while states[(v, tuple(X))][0][0] != 's2':
        route.append(states[(v, tuple(X))][0][0])
        if X != states[(v, tuple(X))][0][1]:
            meetingvehicles = []
            for element in X:
                if element not in states[(v, tuple(X))][0][1]:
                    meetingvehicles.append(element)
            meetingpoints[v] = [meetingvehicles]
        (v, X) = (states[(v, tuple(X))][0][0], states[(v, tuple(X))][0][1])
    if X != states[(v, tuple(X))][0][1]:
        meetingvehicles = []
        for element in X:
            if element not in states[(v, tuple(X))][0][1]:
                meetingvehicles.append(element)
        meetingpoints[v] = [meetingvehicles]
    route.reverse()
    return route, meetingpoints


def generateX1(U, X):
    UmX = []
    for element in U:
        if element not in X:
            UmX.append(element)
    X1 = [[]]
    for n in UmX:
        prev = copy.deepcopy(X1)
        [k.append(n) for k in X1]
        X1.extend(prev)
    return X1


def generateX2(X, subset):
    X2 = []
    for element in X:
        if element not in X2:
            X2.append(element)
    for element in subset:
        if element not in X2:
            X2.append(element)
    X2.sort()
    return X2


def initializeSetPoints(pseed):
    net = sumolib.net.readNet('C:/Users/LSchu/PycharmProjects/SUMO/scenario/lust.net.xml')
    s = '-2836'
    t = '-7626'
    U = ['-1466', '-1720', '-2134', '-28800']
    U.sort()
    # generate vehicles
    outgoing = net.getNode(s).getOutgoing()
    y = random.randint(0, len(outgoing) - 1)
    traci.vehicle.add(lkw, "", "trailer", "now", "random", "random_free", "speedLimit", "current", "max", "current")
    traci.vehicle.setRoute(lkw, ["--30244#0", "-31070#3", "-31070#4"])
    traci.vehicle.changeTarget(lkw, outgoing[y].getID())
    traci.vehicle.moveTo(lkw, outgoing[y].getID() + "_0", 1, 0)
    vehicles[lkw] = []
    f = 0
    while f < len(U):
        outgoing = net.getNode(U[f]).getOutgoing()
        y = random.randint(0, len(outgoing) - 1)
        traci.vehicle.add("Truck_" + str(f + 1), "", "trailer", "now", "random", "random_free", "speedLimit", "current", "max", "current")
        traci.vehicle.setRoute("Truck_" + str(f + 1), ["--30244#0", "-31070#3", "-31070#4"])
        traci.vehicle.changeTarget("Truck_" + str(f + 1), outgoing[y].getID())
        traci.vehicle.moveTo("Truck_" + str(f + 1), outgoing[y].getID() + "_0", 1, 0)
        vehicles["Truck_" + str(f + 1)] = []
        f = f + 1
    junctions = []
    edges = net.getEdges()
    for edge in edges:
        if edge.getFromNode().getID() not in junctions:
            junctions.append(edge.getFromNode().getID())
        if edge.getToNode().getID() not in junctions:
            junctions.append(edge.getToNode().getID())
    # generate packages
    random.seed(pseed)
    f = 0
    while f < p:
        x = random.randint(0, len(junctions) - 1)
        outgoing = net.getNode(junctions[x]).getOutgoing()
        y = random.randint(0, len(outgoing) - 1)
        findroute = traci.simulation.findRoute('-31496#3', outgoing[y].getID())
        if findroute.travelTime != 0:
            NodeCheck = net.getEdge(outgoing[y].getID()).getToNode().getID()
            outgoingCheck = net.getNode(NodeCheck).getOutgoing()
            if len(outgoingCheck) != 1:
                vehicles[lkw].append(Package(target=outgoing[y].getID(), name="Pkg" + str(f + 1)))
                f = f + 1
    return U, s, t


def initialize(pseed):
    net = sumolib.net.readNet('C:/Users/LSchu/PycharmProjects/SUMO/scenario/lust.net.xml')
    junctions = []
    edges = net.getEdges()
    for edge in edges:
        if edge.getFromNode().getID() not in junctions:
            junctions.append(edge.getFromNode().getID())
        if edge.getToNode().getID() not in junctions:
            junctions.append(edge.getToNode().getID())
    # generate vehicles
    sfound = False
    tfound = False
    while not sfound:
        x = random.randint(0, len(junctions) - 1)
        outgoing = net.getNode(junctions[x]).getOutgoing()
        y = random.randint(0, len(outgoing) - 1)
        xpos = traci.junction.getPosition(junctions[x], False)
        FromNodeCheck = net.getEdge(outgoing[y].getID()).getFromNode().getID()
        outgoingCheck = net.getNode(FromNodeCheck).getOutgoing()
        if len(outgoingCheck) != 1:
            if not (4902 < xpos[0] < 8342 and 4223 < xpos[1] < 7641):
                traci.vehicle.add(lkw, "", "trailer", "now", "random", "random_free", "speedLimit", "current", "max", "current")
                traci.vehicle.setRoute(lkw, ["--30244#0", "-31070#3", "-31070#4"])
                traci.vehicle.changeTarget(lkw, outgoing[y].getID())
                traci.vehicle.moveTo(lkw, outgoing[y].getID() + "_0", 1, 0)
                vehicles[lkw] = []
                s = junctions[x]
                sfound = True
    while not tfound:
        x = random.randint(0, len(junctions) - 1)
        outgoing = net.getNode(junctions[x]).getOutgoing()
        y = random.randint(0, len(outgoing) - 1)
        xpos = traci.junction.getPosition(junctions[x], False)
        findroute = traci.simulation.findRoute('-31496#3', outgoing[y].getID())
        if findroute.travelTime != 0:
            if not (4902 < xpos[0] < 8342 and 4223 < xpos[1] < 7641):
                t = junctions[x]
                tfound = True
    U = []
    f = 0
    while f < a:
        x = random.randint(0, len(junctions) - 1)
        outgoing = net.getNode(junctions[x]).getOutgoing()
        y = random.randint(0, len(outgoing) - 1)
        xpos = traci.junction.getPosition(junctions[x], False)
        FromNodeCheck = net.getEdge(outgoing[y].getID()).getFromNode().getID()
        outgoingCheck = net.getNode(FromNodeCheck).getOutgoing()
        if len(outgoingCheck) != 1:
            if 4902 < xpos[0] < 8342 and 4223 < xpos[1] < 7641:
                traci.vehicle.add("Truck_" + str(f + 1), "", "trailer", "now", "random", "random_free", "speedLimit", "current", "max", "current")
                traci.vehicle.setRoute("Truck_" + str(f + 1), ["--30244#0", "-31070#3", "-31070#4"])
                traci.vehicle.changeTarget("Truck_" + str(f + 1), outgoing[y].getID())
                traci.vehicle.moveTo("Truck_" + str(f + 1), outgoing[y].getID() + "_0", 1, 0)
                vehicles["Truck_" + str(f + 1)] = []
                U.append(junctions[x])
                f = f + 1
    U2 = tuple(U)  # tuple, da sonst U2 als Liste dennoch sortiert übergeben wird. Ein Tupel wird nicht sortiert.
    U.sort()
    # generate packages
    random.seed(pseed)
    f = 0
    while f < p:
        x = random.randint(0, len(junctions) - 1)
        outgoing = net.getNode(junctions[x]).getOutgoing()
        y = random.randint(0, len(outgoing) - 1)
        findroute = traci.simulation.findRoute('-31496#3', outgoing[y].getID())
        if findroute.travelTime != 0:
            NodeCheck = net.getEdge(outgoing[y].getID()).getToNode().getID()
            outgoingCheck = net.getNode(NodeCheck).getOutgoing()
            if len(outgoingCheck) != 1:
                vehicles[lkw].append(Package(target=outgoing[y].getID(), name="Pkg" + str(f + 1)))
                f = f + 1
    return U, s, t, U2


def time_convert(sec):
    mins = sec // 60
    sec = sec % 60
    hours = mins // 60
    mins = mins % 60
    print("Time Lapsed = {0}:{1}:{2}".format(int(hours), int(mins), sec))


if __name__ == '__main__':
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
        traci.start(
            ["sumo-gui", "-c", "C:/Users/LSchu/PycharmProjects/SUMO/scenario/dua.actuated.sumocfg", "-b", simstart])
        step()
        run_sim('PyCharm')
        traci.close()
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
