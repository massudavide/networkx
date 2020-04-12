"""
    Beam Search con BackTracking
    effettua una visita in ampiezza, aggiunge i nodi trovati in una mappa chiamata nodiIncotrati ed espande i k migliori
"""
from heapq import heappush, heappop
from itertools import count

import networkx as nx
from networkx.utils import not_implemented_for
from networkx.algorithms.shortest_paths.weighted import _weight_function

__all__ = ['beam_path']


def beam_path(G, source, target, heuristic=None, weight='weight'):
    # se sorgente o destinazione non sono nel grafo inutile cercarli
    if source not in G or target not in G:
        msg = f"Either source {source} or target {target} is not in G"
        raise nx.NodeNotFound(msg)

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    # parametro k che mi dice quanti nodi espandere ogni volta
    k = 20

    push = heappush
    pop = heappop
    weight = _weight_function(G, weight)

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}
    # mappa con nodi incontrati durante il tragitto
    # key : nodo; value : costo, euristica, "padre", marcatore che ci dice se da quel nodo
    # abbiamo espanso i vicini (in caso di backTracking).
    # inizialmente è presente solo il nodo source con marcatore = False perchè ancora non espanso
    nodiIncotrati = {source : (0, heuristic(source, target), None, False)}

    # resto nel primo while finchè posso visitare ancora dei nodi o non ho trovato il target
    while (queue):
        # il secondo while mi serve per togliere tutti i nodi dalla queue
        # riaggiungo nella queueu solo i k migliori
        while (queue) :
            # Pop the smallest item from queue.
            _, __, curnode, dist, parent = pop(queue)

            if curnode == target:
                path = [curnode]
                node = parent
                while node is not None:
                    path.append(node)
                    node = explored[node]
                path.reverse()
                return path

            if curnode in explored:
                # Do not override the parent of starting node
                if explored[curnode] is None:
                    continue

                # Skip bad paths that were enqueued before finding a better one
                qcost, h, _, __ = nodiIncotrati[curnode]
                if qcost < dist:
                    continue

            explored[curnode] = parent

            # ogni volta espando un nodo lo marco come True
            # per non utilizzarlo in caso di backTracking
            costo, h, padre, _ = nodiIncotrati[curnode]
            nodiIncotrati[curnode] = costo, h, padre, True

            # espando i neighbor del nodo corrente e li aggiungo nella mappa "enqueued"
            for neighbor, w in G[curnode].items():
                ncost = dist + weight(curnode, neighbor, w)
                h = heuristic(neighbor, target)
                # se non ho mai visto questo neighbor allora lo aggiungo a nodiIncotrati
                if neighbor not in nodiIncotrati :
                    nodiIncotrati[neighbor] = ncost, h, curnode, False
                    enqueued[neighbor] = ncost, h, curnode
                # else: devo controllare se il costo è minore e in caso lo riscrivo
                else :
                    costo, _, __, marker = nodiIncotrati[neighbor]
                    if costo > ncost and marker == False :
                        nodiIncotrati[neighbor] = ncost, h, curnode, marker
                        enqueued[neighbor] = ncost, h, curnode

        # la queue ora è vuota

        # ordino la mappa dei neighbor secondo euristica
        ordinato = sorted(enqueued.items(), key=lambda x: x[1][1])

        # se k è più grande del numero di nodi in ordinato
        # ci sono due casi: o il numero totale di fratelli è minore di k, oppure sto in k foglie
        # in ogni caso faccio uso di backTracking
        if(len(ordinato) < k) :
            # prendi "nodiIncotrati" e crea una lista ordinata secondo euristica
            lista = sorted(nodiIncotrati.items(), key=lambda x: x[1][1])
            # aggiungi in "ordinato" gli n nodi fino a che len(ordinato) == k
            for nodo in lista :
                if(len(ordinato) >= k) :
                    break
                # se il nodo non è stato espanso e non è presente in "enqueued" lo aggiungo alla lista
                if nodo[1][3] == False and nodo[0] not in  enqueued :
                    ordinato.append(nodo)

            # se sono entrato nell'if i nodi non sono più ordinati
            ordinato = sorted(ordinato, key=lambda x: x[1][1])

        # ripulisco la mappa in modo da poter caricare i neighbor nell'iterazione successiva
        enqueued.clear()

        # ciclo k volte e faccio push degli elementi con euristica minore
        for x in range(k) :
            # se non ho più elementi in "ordinato" inutile continuare il for
            if(len(ordinato) - 1 < x) :
                break
            push(queue, (ncost + h, next(c), ordinato[x][0], ncost, ordinato[x][1][2]))

    # se la queue è vuota allora ho visitato tutti i nodi senza mai trovare il target...posso pure fermarmi
    raise nx.NetworkXNoPath(f"Node {target} not reachable from {source}")
