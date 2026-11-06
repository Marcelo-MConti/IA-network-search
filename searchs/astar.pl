:- include('../graph.pl').

% caso base
astar_busca(Vf, _, _, [(_-Dist-(Vf-CaminhoAtual)) | _], Caminho, Dist) :-
    reverse(CaminhoAtual, Caminho).

astar_busca(Vf, Vertices, Arestas, Fila, Caminho, Dist) :-
    Fila = [(F-DistAtual-(Atual-CaminhoAtual)) | Resto],

    % encontrando todos os vizinhos do nó Atual
    findall(
        F-D-(Id-[Id | CaminhoAtual]),
        (
            ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, CaminhoAtual),
            D is DistAtual + P,
            member(v(Id, X1, Y1), Vertices),
            member(v(Vf, X2, Y2), Vertices),
            distancia(v(Id, X1, Y1), v(Vf, X2, Y2), H),
            F is DistAtual + P + H
        ),
        Novos
    ),

    % colocando na fila
    append(Novos, Resto, NovaFila),

    % ordenando a fila pela distância
    keysort(NovaFila, FilaOrdenada),
    astar_busca(Vf, Vertices, Arestas, FilaOrdenada, Caminho, Dist).

astar(Vi, Vf, N, K, P, Caminho, Dist) :-
    gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
    member(v(Vi, _, _), Vertices),
    member(v(Vf, _, _), Vertices),
    astar_busca(Vf, Vertices, Arestas, [(0-0-(Vi-[Vi]))], Caminho, Dist).



