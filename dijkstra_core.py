# dijkstra_core.py
# Implementación del Algoritmo de Dijkstra con heapq
# Proyecto: Optimización de Rutas LAN | Grupo 7 | ESPOCH 2026

import heapq
import networkx as nx
import matplotlib.pyplot as plt

# ── Construcción del grafo ──────────────────────────────────
def construir_red_lan():
    """Construye el grafo LAN de 20 nodos con pesos OSPF."""
    G = nx.Graph()

    # Agregar nodos etiquetados por zona
    zonas = {0:'Router', 1:'SW-Admin1', 2:'SW-Admin2',
             3:'PC-A1', 4:'PC-A2', 5:'Impresora', 6:'NAS',
             7:'SW-Lab1', 8:'SW-Lab2', 9:'PC-L1',
             10:'PC-L2', 11:'PC-L3', 12:'AP-Lab', 13:'Camara',
             14:'CoreSW', 15:'Servidor1', 16:'Servidor2',
             17:'Firewall', 18:'Backup', 19:'DMZ'}
    for nodo, nombre in zonas.items():
        G.add_node(nodo, label=nombre)

    # Aristas con ancho de banda (Mbps) → costo OSPF = 10^8 / BW
    enlaces = [  # (nodo_u, nodo_v, BW_Mbps)
        (0,1,1000),(0,7,1000),(0,14,10000),
        (1,2,100),(1,3,100),(1,4,100),(1,5,100),
        (2,6,100),(2,7,1000),(7,8,1000),
        (7,9,100),(7,10,100),(8,11,100),
        (8,12,150),(8,13,100),(14,15,10000),
        (14,16,10000),(14,17,10000),(14,18,1000),
        (14,19,1000),(15,16,10000),(17,18,1000),
        (17,19,1000),(0,2,1000),(0,8,1000),
        (1,7,1000),(2,8,1000),(6,14,1000),
        (9,10,100),(10,11,100),(12,13,100),
        (15,17,1000),(16,18,1000),(19,15,1000) ]

    for u, v, bw in enlaces:
        costo = round(1e8 / (bw * 1e6), 4)  # Costo OSPF
        G.add_edge(u, v, weight=costo, bandwidth=bw)
    return G

# ── Algoritmo de Dijkstra ───────────────────────────────────
def dijkstra(grafo, fuente):
    """
    Dijkstra con montículo binario. Complejidad: O((V+E)logV).
    Retorna: (distancias, predecesores)
    """
    dist = {v: float('inf') for v in grafo.nodes()}
    pred = {v: None for v in grafo.nodes()}
    dist[fuente] = 0
    cola = [(0, fuente)]  # (costo_acumulado, nodo)

    while cola:
        costo_u, u = heapq.heappop(cola)  # Extraer mínimo
        if costo_u > dist[u]:  # Nodo ya procesado, omitir
            continue
        # Relajar aristas adyacentes
        for v in grafo.neighbors(u):
            w = grafo[u][v]['weight']
            if dist[u] + w < dist[v]:  # Condición de relajación
                dist[v] = dist[u] + w
                pred[v] = u
                heapq.heappush(cola, (dist[v], v))
    return dist, pred

def reconstruir_ruta(pred, fuente, destino):
    """Reconstruye el camino óptimo desde pred[]."""
    ruta, nodo = [], destino
    while nodo is not None:
        ruta.append(nodo)
        nodo = pred[nodo]
    return list(reversed(ruta))  # fuente → destino

# ── Visualización ──────────────────────────────────────────
def visualizar_ruta(G, ruta, dist, fuente, destino):
    pos = nx.spring_layout(G, seed=42)
    aristas_ruta = list(zip(ruta, ruta[1:]))
    labels = nx.get_node_attributes(G, 'label')
    plt.figure(figsize=(14, 10))
    # Dibujar toda la red en gris
    nx.draw_networkx_nodes(G, pos, node_color='lightgray',
                           node_size=600)
    nx.draw_networkx_edges(G, pos, edge_color='gray',
                           alpha=0.4, width=1)
    # Resaltar ruta óptima en azul
    nx.draw_networkx_nodes(G, pos, nodelist=ruta,
                           node_color='steelblue', node_size=800)
    nx.draw_networkx_edges(G, pos, edgelist=aristas_ruta,
                           edge_color='steelblue', width=3)
    nx.draw_networkx_labels(G, pos, labels, font_size=8)
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels, font_size=7)
    plt.title(f'Ruta óptima {fuente}→{destino} | Costo={dist[destino]:.4f}')
    plt.savefig('ruta_optima.png', dpi=150, bbox_inches='tight')
    plt.show()

# ── Ejecución principal ─────────────────────────────────────
if __name__ == '__main__':
    G = construir_red_lan()
    fuente, destino = 0, 19  # Router → DMZ
    dist, pred = dijkstra(G, fuente)
    ruta = reconstruir_ruta(pred, fuente, destino)
    print(f'Ruta óptima: {ruta}')
    print(f'Costo total: {dist[destino]:.4f}')
    visualizar_ruta(G, ruta, dist, fuente, destino)