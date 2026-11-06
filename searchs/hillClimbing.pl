:- include('../graph.pl').

% caso base
hill_climb_busca(Vf, Vf, _, Visitados, Caminho, Dist, Dist):-
  reverse(Visitados, Caminho).

hill_climb_busca(Atual, Vf, Arestas, Visitados, Caminho, DistAc, Dist) :-

  % encontrando todos os vizinhos no node Atual
  findall(
    P-Id, 
    (
      (member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
      not(member(Id, Visitados))
    ),
    Candidatos),

  % Ordenando os novos nodes de forma ascendente
  keysort(Candidatos, CandidatosOrdenados),
  
  % escolhendo o menor vizinho local para seguir
  CandidatosOrdenados = [(P-NovoNode) | _],

  DistNova is DistAc + P,

  hill_climb_busca(NovoNode, Vf, Arestas, [NovoNode | Visitados], Caminho, DistNova, Dist).

hill_climb(Vi, Vf, N, K, P, Caminho, Dist) :-
  gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
  member(v(Vi, _, _), Vertices),
  member(v(Vf, _, _), Vertices),
  hill_climb_busca(Vi, Vf, Arestas, [Vi], Caminho, 0, Dist).



