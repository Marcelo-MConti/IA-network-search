% a. (n = 2500, k = 7, p = 0.10)
% b. (n = 2500, k = 7, p = 0.05)
% c. (n = 2500, k = 7, p = 0.01)

% Adicionar verificação de duplicata de pontos/vértices
:- use_module(library(random)).
:- use_module(library(lists)).

% Gera N vertices com coordenadas aleatorias.
gera_vertices(N, Vertices) :-
    gera_vertices2(0, N, [], Vert),
    reverse(Vert, Vertices).

gera_vertices2(N, N, Vertices, Vertices).

gera_vertices2(I, N, Vert_parcial, Vertices) :-
    repeat,
    random_between(0, N, X),
    random_between(0, N, Y),
    \+ member(v(_, X, Y), Vert_parcial),
    !,
    NextI is I + 1,
    gera_vertices2(NextI, N, [v(I, X, Y) | Vert_parcial], Vertices).

% Calcula a distancia euclidiana.
distancia(v(_, X1, Y1), v(_, X2, Y2), sqrt((X1-X2)^2 + (Y1-Y2)^2)).

% Padroniza a aresta para evitar duplicatas.
aresta(U, V, D, e(U, V, D)) :-
    U < V, !.
aresta(U, V, D, e(V, U, D)).

pega_k(0, _, []) :- !.
pega_k(_, [], []) :- !.
pega_k(K, [H|T], [H|Resultado]) :-
    K1 is K - 1,
    pega_k(K1, T, Resultado).

vizinhos_proximos(V, Vertices, K, Arestas) :-
    % D-Id é a distânica e o id do vértice
    findall(D-Id,
        ( % Para o vértice atual, percorremos todos os vértices da rede
          member(W, Vertices),
          V = v(I, _, _),
          W = v(Id, _, _),
          I =\= Id, % ignorar próprio vértice
          distancia(V, W, D)
        ),
        Distancias),
    keysort(Distancias, Ordenadas),
    % seleciona k vizinhos mais próximos
    pega_k(K, Ordenadas, MaisProximos),
    % cria arestas
    findall(e(U, W, D),
        ( member(D-Id, MaisProximos),
          V = v(I, _, _),
          aresta(I, Id, D, e(U, W, D))
        ),
        Arestas).

gera_arestas(Vertices, K, Arestas) :-
    findall(A,
        ( % seleciona um vértice por vez
          member(V, Vertices),
          vizinhos_proximos(V, Vertices, K, Lista),
          member(A, Lista)
        ),
        Todas),
    sort(Todas, Arestas).

gera_rede_knn(N, K, Vertices, Arestas) :-
    gera_vertices(N, Vertices),
    gera_arestas(Vertices, K, Arestas).

reconecta_arestas(Arestas, Vertices, P, NovasArestas) :-
    % percorre a lista de Arestas chamando reconecta para cada elem
    maplist(reconecta(Vertices, Arestas, P), Arestas, Resultado),
    sort(Resultado, NovasArestas). % ordena e remove duplicatas

reconecta(Vertices, Arestas, P, e(U, V, Peso), NovaAresta) :-
    random(R),
    ( R < P ->
        % Mantem U e sorteia outro vertice para substituir V.
        findall(Id,
            ( member(v(Id, _, _), Vertices),
              Id =\= U,
              not(member(e(U, Id, _), Arestas))
            ),
            Candidatos),
        random_member(W, Candidatos),
        member(v(U, X1, Y1), Vertices),
        member(v(W, X2, Y2), Vertices),
        D is sqrt((X1-X2)^2 + (Y1-Y2)^2),
        aresta(U, W, D, NovaAresta)
    ;
        NovaAresta = e(U, V, Peso),
        % O peso original e preservado quando a aresta nao muda.
        true
    ).

gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas) :-
    gera_rede_knn(N, K, Vertices, KNN),
    reconecta_arestas(KNN, Vertices, P, Arestas).

% Executar
% gera_rede_pequeno_mundo(2500, 7, 0.05, V, E)