:- ensure_loaded('graph.pl').

% --- DFS ---
dfs(Vi,Vf,Vertices,Arestas,Caminho,Distancia):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    dfs_busca(Vf,Vertices,Arestas,[0-(Vi-[Vi])],Caminho,[Vi],Distancia).

% Caso em que chegamos ao vertice final
dfs_busca(Vf,_,_,[Dist-(Vf-CaminhoAtual)|_],Caminho,_,Dist) :-
    reverse(CaminhoAtual,Caminho), !.

dfs_busca(Vf,Vertices,Arestas,[Dist-(Atual-CaminhoAtual) | DemaisCaminhos],Caminho,Fechados,Distancia):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já visitados (conjunto fechado) para manter busca limitada
    exclude(member_right(Fechados), L, Lfiltrado),
    adicionar_caminhos_dfs(Lfiltrado,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices),
    dfs_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados],Distancia).

% Responsável por encontrar os filhos da aresta Atual
filhos(Atual,Arestas,CaminhoAtual,L):-
    findall(
        X,
        (
            (% Todos os Vertices que se conectam com Atual
                member(e(X,Atual,_),Arestas);
                member(e(Atual,X,_),Arestas)
            ), 
            not(member(X,CaminhoAtual)) % Os vertices novos n podem estar no caminho
        ),
        L 
    ).

% member_right/2: X é membro da segunda lista (para exclude).
member_right(Lista, X) :- member(X, Lista).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_dfs(L,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices):-
    findall(
        NovaDist-(X-[X|CaminhoAtual]),
        (
            member(X,L),
            member(v(X,X1,Y1),Vertices),
            member(v(Atual,AtualX,AtualY),Vertices),
            distancia(v(Atual,AtualX,AtualY),v(X,X1,Y1),Add),
            NovaDist is Dist + Add
        ),
        CaminhosNovos
    ),
    append(CaminhosNovos,DemaisCaminhos,CaminhosAtualizados).


% --- BFS ---
bfs(Vi,Vf,Vertices,Arestas,Caminho,Distancia):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    bfs_busca(Vf,Vertices,Arestas,[0-(Vi-[Vi])],Caminho,[Vi],[Vi],Distancia).

% Caso em que chegamos ao vertice final
bfs_busca(Vf,_,_,[Dist-(Vf-CaminhoAtual)|_],Caminho,_,_,Dist) :-
    reverse(CaminhoAtual,Caminho), !.

bfs_busca(Vf,Vertices,Arestas,[Dist-(Atual-CaminhoAtual) | DemaisCaminhos],Caminho,Fechados,Visitados,Distancia):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já expandidos ou já enfileirados (evita duplicatas na fila)
    exclude(member_right(Fechados), L, L1),
    exclude(member_right(Visitados), L1, Lfiltrado),
    adicionar_caminhos_bfs(Lfiltrado,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices),
    % registra os novos enfileirados para não reenfileirá-los depois
    findall(X, member(X, Lfiltrado), NovosVisitados),
    append(NovosVisitados, Visitados, VisitadosNovo),
    bfs_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados],VisitadosNovo,Distancia).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_bfs(L,Atual,Dist,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Vertices):-
    findall(
        NovaDist-(X-[X|CaminhoAtual]),
        (
            member(X,L),
            member(v(X,X1,Y1),Vertices),
            member(v(Atual,AtualX,AtualY),Vertices),
            distancia(v(Atual,AtualX,AtualY),v(X,X1,Y1),Add),
            NovaDist is Dist + Add
        ),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,CaminhosAtualizados).


% --- BestFirst --- 
best_first(Vi,Vf,Vertices,Arestas,Caminho,Distancia):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    best_first_busca(Vf,Vertices,Arestas,[0-(0-(Vi-[Vi]))],Caminho,Distancia).

best_first_busca(Vf,_,_,[_-(Dist-(Vf-CaminhoAtual))|_], Caminho,Distancia):-
    reverse(CaminhoAtual,Caminho),
    Distancia=Dist.

best_first_busca(Vf,Vertices,Arestas,[(H-(Dist-(Atual-CaminhoAtual)))| DemaisCaminhos],Caminho,Distancia):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    adicionar_caminhos_best(Vf,Vertices,L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Atual,Dist),
    best_first_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho,Distancia).

adicionar_caminhos_best(Vf,Vertices,L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Atual,Dist):-
    findall(
        D-(NovaDist-(X-[X|CaminhoAtual])),
        (
            member(X,L),
            member(v(X,X1,Y1),Vertices),
            member(v(Vf,X2,Y2),Vertices),
            member(v(Atual,AtualX,AtualY),Vertices),
            distancia(v(X,X1,Y1),v(Vf,X2,Y2),D),
            distancia(v(Atual,AtualX,AtualY),v(X,X1,Y1),Add),
            NovaDist is Dist + Add
        ),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,S),
    keysort(S,CaminhosAtualizados).

    
% --- Dijkstra --- 
% Caso base: o destino foi retirado da fila com a menor distancia.
dijkstra_busca(Vf, _, [(Dist-(Vf-CaminhoAtual)) | _],
               Caminho, Dist, _) :-
    reverse(CaminhoAtual, Caminho), !.

% Ignora entradas antigas de vertices que ja foram expandidos.
dijkstra_busca(Vf, Arestas,
               [(DistAtual-(Atual-CaminhoAtual)) | Resto],
               Caminho, Dist, Fechados) :-

    ( member(Atual, Fechados) ->
        dijkstra_busca(Vf, Arestas, Resto,
                       Caminho, Dist, Fechados)

    ;
        % Encontra os vizinhos ainda nao fechados.
        findall(
            D-(Id-[Id | CaminhoAtual]),
            (
                ( member(e(Atual, Id, P), Arestas)
                ; member(e(Id, Atual, P), Arestas)
                ),
                \+ member(Id, [Atual | Fechados]),
                D is DistAtual + P
            ),
            Novos
        ),

        % Insere os novos caminhos na fila.
        append(Novos, Resto, NovaFila),

        % Ordena a fila pela distancia acumulada.
        keysort(NovaFila, FilaOrdenada),

        dijkstra_busca(Vf, Arestas, FilaOrdenada,
                       Caminho, Dist, [Atual | Fechados])
    ).

dijkstra(Vi, Vf, Vertices, Arestas, Caminho, Dist) :-
    member(v(Vi, _, _), Vertices),
    member(v(Vf, _, _), Vertices),
    dijkstra_busca(Vf, Arestas,
                   [(0-(Vi-[Vi]))],
                   Caminho, Dist, []).


% --- hillClimbing ---
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

hill_climb(Vi, Vf, Vertices, Arestas, Caminho, Dist) :-
  member(v(Vi, _, _), Vertices),
  member(v(Vf, _, _), Vertices),
  hill_climb_busca(Vi, Vf, Arestas, [Vi], Caminho, 0, Dist).

% --- Astar ---
% caso base
astar_busca(Vf, _, _, [(_-Dist-(Vf-CaminhoAtual)) | _], Caminho, Dist, _Fechados) :-
    reverse(CaminhoAtual, Caminho), !.

astar_busca(Vf, Vertices, Arestas, Fila, Caminho, Dist, Fechados) :-
    Fila = [(_Fn-DistAtual-(Atual-CaminhoAtual)) | Resto],

    % encontrando todos os vizinhos do nó Atual (não visitados e não já
    % expandidos - um conjunto fechado evita re-expansão infinita e, como a
    % heurística é admissível e consistente, mantém a otimalidade)
    findall(
        Fn-Dn-(Id-[Id | CaminhoAtual]),
        (
            ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, CaminhoAtual),
            \+ member(Id, Fechados),
            Dn is DistAtual + P,
            member(v(Id, X1, Y1), Vertices),
            member(v(Vf, X2, Y2), Vertices),
            distancia(v(Id, X1, Y1), v(Vf, X2, Y2), H),
            Fn is DistAtual + P + H
        ),
        Novos
    ),

    % colocando na fila
    append(Novos, Resto, NovaFila),

    % ordenando a fila pela distância
    keysort(NovaFila, FilaOrdenada),
    astar_busca(Vf, Vertices, Arestas, FilaOrdenada,
                Caminho, Dist, [Atual | Fechados]).

astar(Vi, Vf, Vertices, Arestas, Caminho, Dist) :-
    member(v(Vi, _, _), Vertices),
    member(v(Vf, _, _), Vertices),
    astar_busca(Vf, Vertices, Arestas, [(0-0-(Vi-[Vi]))], Caminho, Dist, [Vi]).

