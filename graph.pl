% a. (n = 2500, k = 7, p = 0.10)
% b. (n = 2500, k = 7, p = 0.05)
% c. (n = 2500, k = 7, p = 0.01)

:- use_module(library(random)).
:- use_module(library(lists)).

% Gera N vértices com coordenadas aleatórias.
gera_vertices(N, Vertices) :-
    gera_vertices2(0, N, [], Vert),
    % precisa inverter pq os vértices foram colocados como uma pilha (maior índice está antes)
    reverse(Vert, Vertices).

% Quando n vértices foram produzidos, para
gera_vertices2(N, N, Vertices, Vertices).

% enquanto está em i<n, produz novo vértice e coloca na cabeça da lista
gera_vertices2(I, N, Vert_parcial, Vertices) :-
    repeat,
    random_between(0, N, X),
    random_between(0, N, Y),
    % evitar que o novo vértice sobreponha um já existente
    \+ member(v(_, X, Y), Vert_parcial),
    !,
    NextI is I + 1,
    gera_vertices2(NextI, N, [v(I, X, Y) | Vert_parcial], Vertices).

% Calcula a distancia euclidiana. Operador 'is' é necessário para o prolog realizar a operação aritmética
distancia(v(_, X1, Y1), v(_, X2, Y2), D) :- D is sqrt((X1-X2)^2 + (Y1-Y2)^2).

% Padroniza a aresta para evitar duplicatas e colocar o vértice de menor id primeiro.
aresta(U, V, D, e(U, V, D)) :-
    U < V, !.
aresta(U, V, D, e(V, U, D)).

% k primeiros elementos de uma lista
pega_k(0, _, []) :- !.
pega_k(_, [], []) :- !.
pega_k(K, [H|T], [H|Resultado]) :-
    K1 is K - 1,
    pega_k(K1, T, Resultado).

% encontra os k-mais-próximos vizinhos, conecta com V e coloca em Arestas
vizinhos_proximos(V, Vertices, K, Arestas) :-
    % I é o id de V
    V = v(I, _, _),

    % D-Id é a distânica e o id do vizinho
    findall(D-Id,
        ( % Para o vértice atual, percorremos todos os vértices da rede
          member(W, Vertices),
          W = v(Id, _, _),
          I =\= Id, % ignorar próprio vértice
          distancia(V, W, D)
        ),
        Distancias_IDs),
    keysort(Distancias_IDs, Ordenadas),

    % seleciona k vizinhos mais próximos
    pega_k(K, Ordenadas, MaisProximos),

    % cria as arestas com os mais próximos
    findall(Aresta,
        ( member(D-Id, MaisProximos),
          aresta(I, Id, D, Aresta)
        ),
        Arestas).

gera_arestas(Vertices, K, Arestas) :-
    findall(Aresta,
        ( % seleciona um vértice por vez e coloca as arestas para os k-próximos em Todas
          member(V, Vertices),
          vizinhos_proximos(V, Vertices, K, Proximos),
          member(Aresta, Proximos)
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
