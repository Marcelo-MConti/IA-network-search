:- ensure_loaded('../graph.pl').

% caso base
astar_busca(Vf, _, _, [(_-Dist-(Vf-CaminhoAtual)) | _], Caminho, Dist, _Fechados) :-
    reverse(CaminhoAtual, Caminho), !.

astar_busca(Vf, Vertices, Arestas, Fila, Caminho, Dist, Fechados) :-
    Fila = [(F_atual-DistAtual-(Atual-CaminhoAtual)) | Resto],

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

astar(Vi, Vf, N, K, P, Caminho, Dist) :-
    gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas),
    member(v(Vi, _, _), Vertices),
    member(v(Vf, _, _), Vertices),
    astar_busca(Vf, Vertices, Arestas, [(0-0-(Vi-[Vi]))], Caminho, Dist, [Vi]).



