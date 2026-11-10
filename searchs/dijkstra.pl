:- ensure_loaded('../graph.pl').

% caso base
dijkstra_busca(Vf, _, [(Dist-(Vf-CaminhoAtual)) | _], Caminho, Dist, _Fechados, _Enfileirados) :-
    reverse(CaminhoAtual, Caminho), !.

dijkstra_busca(Vf, Arestas, Fila, Caminho, Dist, Fechados, Enfileirados) :-
    Fila = [(DistAtual-(Atual-CaminhoAtual)) | Resto],

    % encontrando todos os vizinhos do nó Atual (ignorando nós já expandidos e
    % nós já enfileirados para evitar que a fila cresça exponencialmente com nós duplicados)
    findall(
        D-(Id-[Id | CaminhoAtual]),
        (
            ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, CaminhoAtual),
            \+ member(Id, Fechados),
            \+ member(Id, Enfileirados),
            D is DistAtual + P
        ),
        Novos
    ),

    % colocando na fila
    append(Novos, Resto, NovaFila),

    % ordenando a fila pela distância
    keysort(NovaFila, FilaOrdenada),

    % atualiza o conjunto de enfileirados com os novos ids
    findall(Id, member(_-(Id-_), Novos), NovosIds),
    append(NovosIds, Enfileirados, EnfileiradosNovo),

    dijkstra_busca(Vf, Arestas, FilaOrdenada, Caminho, Dist,
                   [Atual | Fechados], EnfileiradosNovo).

dijkstra(Vi, Vf, Vertices, Arestas, Caminho, Dist) :-
    member(v(Vi, _, _), Vertices),
    member(v(Vf, _, _), Vertices),
    dijkstra_busca(Vf, Arestas, [(0-(Vi-[Vi]))], Caminho, Dist, [Vi], [Vi]).



