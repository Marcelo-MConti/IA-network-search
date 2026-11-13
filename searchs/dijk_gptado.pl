
:- ensure_loaded('../graph.pl').

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

