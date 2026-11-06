:- include('../graph.pl').

% caso base
dijkstra_busca(Vf, Vf, _, [(Vf, Dist)|_], Visitados, [Vf | Visitados], Dist).

dijkstra_busca(Atual, Vf, Arestas, Fila, Visitados, Caminho, Dist) :-

  Fila = [(Atual, DistAtual) | _],

  % encontrando todos os vizinhos no node Atual
  findall(
    D-Id, 
    (
      member(arestas(Atual, Id, P, _), Arestas),
      not(member(Id, Visitados)),
      D is DistAtual + P
    ),
    Novos),

  % Ordenando os novos nodes de forma ascendente
  keysort(Novos, NovosOrdenados),

  % colocando na Fila
  append(NovosOrdenados, Fila, NovaFila),

  NovaFila = [(NovoNode, _) | _],

  dijkstra_busca(NovoNode, Vf, Arestas, NovaFila, [Atual | Visitados], Caminho, Dist).

dijkstra(Vi, Vf, N, K, P, Caminho, Dist) :-
  gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
  member(v(Vi, _, _), Vertices),
  member(v(Vf, _, _), Vertices),
  dijkstra_busca(Vi, Vf, Arestas, [(Vi, 0)], [Vi], Cam, Dist),
  reverse(Cam, Caminho).



