:- include('../graph.pl').

% caso base
dijkstra_busca(Vf, _, [(Dist-(Vf-CaminhoAtual)) | _], Caminho, Dist) :-
    reverse(CaminhoAtual, Caminho).

dijkstra_busca(Vf, Arestas, Fila, Caminho, Dist) :-
    Fila = [(DistAtual-(Atual-CaminhoAtual)) | Resto],

    % encontrando todos os vizinhos do nó Atual
    findall(
        D-(Id-[Id | CaminhoAtual]),
        (
            ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, CaminhoAtual),
            D is DistAtual + P
        ),
        Novos
    ),

    % colocando na fila
    append(Novos, Resto, NovaFila),

    % ordenando a fila pela distância
    keysort(NovaFila, FilaOrdenada),
    dijkstra_busca(Vf, Arestas, FilaOrdenada, Caminho, Dist).

dijkstra(Vi, Vf, N, K, P, Caminho, Dist) :-
    gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
    member(v(Vi, _, _), Vertices),
    member(v(Vf, _, _), Vertices),
    dijkstra_busca(Vf, Arestas, [(0-(Vi-[Vi]))], Caminho, Dist).



