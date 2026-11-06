:- include('../graph.pl').

% caso base
dijkstra_busca(Vf, _, [(Dist-Vf)|_], [Vf | C], Caminho, Dist) :-
  reverse([Vf | C], Caminho),
  !.

dijkstra_busca(Vf, Arestas, Fila, Visitados, Caminho, Dist) :-

  Fila = [(DistAtual-Atual) | C],

  FilaSemHead = C, 

  % encontrando todos os vizinhos no node Atual
  findall(
    D-Id, 
    (
      member(aresta(Atual, Id, P, _), Arestas),
      not(member(Id, Visitados)),
      D is DistAtual + P
    ),
    Novos),

  % colocando na Fila
  append(Novos, FilaSemHead, NovaFila),
  
  % Ordenando a fila pela distancia
  keysort(NovaFila, FilaOrdenada),

  FilaOrdenada = [(_-NovoNode) | _],

  dijkstra_busca(Vf, Arestas, FilaOrdenada, [NovoNode | Visitados], Caminho, Dist).

dijkstra(Vi, Vf, N, K, P, Caminho, Dist) :-
  gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
  member(v(Vi, _, _), Vertices),
  member(v(Vf, _, _), Vertices),
  dijkstra_busca(Vf, Arestas, [(0-Vi)], [Vi], Caminho, Dist).



