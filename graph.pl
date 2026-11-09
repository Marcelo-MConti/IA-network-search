% a. (n = 2500, k = 7, p = 0.10)
% b. (n = 2500, k = 7, p = 0.05)
% c. (n = 2500, k = 7, p = 0.01)

:- use_module(library(random)).
:- use_module(library(lists)).

% Gera N vértices com coordenadas aleatórias
gera_vertices(N, Vertices) :-
    gera_vertices2(0, N, [], Vert),
    % É necessário inverter porque os vértices foram inseridos no início da lista
    reverse(Vert, Vertices).

% Para quando n vértices forem produzidos
gera_vertices2(N, N, Vertices, Vertices).

% Enquanto I < N, produz novo vértice e coloca na cabeça da lista
gera_vertices2(I, N, Vert_parcial, Vertices) :-
    repeat,

    % Sorteia coordenadas entre 0 e N
    random_between(0, N, X),
    random_between(0, N, Y),

    % Evita que o novo vértice sobreponha um já existente
    \+ member(v(_, X, Y), Vert_parcial),

    % Interrompe o repeat quando encontra coordenadas válidas
    !,

    NextI is I + 1,

    % Insere o novo vértice no início da lista
    gera_vertices2(NextI, N, [v(I, X, Y) | Vert_parcial], Vertices).

% Calcula a distância euclidiana
distancia(v(_, X1, Y1), v(_, X2, Y2), D) :- 
    D is sqrt((X1-X2)^2 + (Y1-Y2)^2).

% Padroniza a aresta para evitar duplicatas e colocar o vértice de menor id primeiro
aresta(U, V, D, e(U, V, D)) :-
    U < V, !.
aresta(U, V, D, e(V, U, D)).

% k primeiros elementos de uma lista
pega_k(0, _, []) :- !.

pega_k(_, [], []) :- !.

pega_k(K, [H|T], [H|Resultado]) :-
    K1 is K - 1,
    pega_k(K1, T, Resultado).

% Encontra os k-mais-próximos vizinhos, conecta com V e coloca em Arestas
vizinhos_proximos(V, Vertices, K, Arestas) :-
    % Obtém o id (I) do vértice atual
    V = v(I, _, _),

    % D-Id é a distânica e o id do vizinho
    findall(D-Id,
        ( % Para o vértice atual, percorremos todos os vértices da rede
          member(W, Vertices),

          % Obtém o ID do vértice candidato
          W = v(Id, _, _),

          % Não considera o próprio vértice como vizinho
          I =\= Id, 

          distancia(V, W, D)
        ),
        Distancias_IDs),
    % Ordena os pares pela distância
    keysort(Distancias_IDs, Ordenadas),

    % Seleciona os K vizinhos mais próximos
    pega_k(K, Ordenadas, MaisProximos),

    % Cria as arestas com os mais próximos
    findall(Aresta,
        ( member(D-Id, MaisProximos),
          aresta(I, Id, D, Aresta)
        ),
        Arestas).

gera_arestas(Vertices, K, Arestas) :-
    findall(Aresta,
        ( % Seleciona um vértice por vez e coloca as arestas para os k-próximos em todas
          member(V, Vertices),
          vizinhos_proximos(V, Vertices, K, Proximos),
          
          % Adiciona cada aresta encontrada à lista
          member(Aresta, Proximos)
        ),
        Todas),
    % Ordena as arestas e remove duplicatas
    sort(Todas, Arestas).

gera_rede_knn(N, K, Vertices, Arestas) :-
    gera_vertices(N, Vertices),
    gera_arestas(Vertices, K, Arestas).

reconecta_arestas(Arestas, Vertices, P, NovasArestas) :-
    % Percorre a lista de Arestas chamando reconecta para cada elemento
    maplist(reconecta(Vertices, Arestas, P), Arestas, Resultado),
    
    % Ordena e remove duplicatas
    sort(Resultado, NovasArestas). 

reconecta(Vertices, Arestas, P, e(U, V, Peso), NovaAresta) :-
    random(R),

    ( R < P ->
        % Mantem U e sorteia outro vertice para substituir V
        findall(Id,
            ( member(v(Id, _, _), Vertices),

              % Não permite conectar U com ele mesmo
              Id =\= U,

              % Padroniza a aresta para verificar se já existe
              aresta(U, Id, _, Candidata),

              % Não permite criar uma aresta já existente
              \+ member(Candidata, Arestas)
            ),
            Candidatos),

        % Se não houver candidatos, mantém a aresta original
        (
            Candidatos = [] ->
                NovaAresta = e(U, V, Peso)
        ;
            % Sorteia um novo vértice
            random_member(W, Candidatos),

            % Recupera as coordenadas dos vértices
            member(v(U, X1, Y1), Vertices),
            member(v(W, X2, Y2), Vertices),

            % Calcula a nova distância
            D is sqrt((X1-X2)^2 + (Y1-Y2)^2),
            
            % Cria a nova aresta
            aresta(U, W, D, NovaAresta)
        )
    ;
        % Caso contrário, mantém a aresta original
        NovaAresta = e(U, V, Peso)
    ).

% ---------------------------------------------------------------------------
% Deterministic random seeding.
% The graph is generated from a fixed, reproducible seed so that every call to
% gera_rede_pequeno_mundo/5 (including repeated calls from the search
% algorithms) produces the SAME small-world network for the same N, K and P.
% Change the seed with set_graph_seed/1 if you want a different but still
% reproducible instance.
% ---------------------------------------------------------------------------
:- dynamic graph_seed/1.
graph_seed(31).

set_graph_seed(S) :-
    retractall(graph_seed(_)),
    assert(graph_seed(S)).

gera_rede_pequeno_mundo(N, K, P, Vertices, Arestas) :-
    graph_seed(S),
    set_random(seed(S)),
    gera_rede_knn(N, K, Vertices, KNN),
    reconecta_arestas(KNN, Vertices, P, Arestas).

% --- Executar ---
% gera_rede_pequeno_mundo(2500, 7, 0.10, V, E).

% gera_rede_pequeno_mundo(2500, 7, 0.05, V, E).

% gera_rede_pequeno_mundo(2500, 7, 0.01, V, E).