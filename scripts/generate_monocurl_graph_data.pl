% generate_monocurl_graph_data.pl
%
% Driver to build a small-world graph and run all six search algorithms
% (Dijkstra, A*, Hill Climbing, Best-First, BFS, DFS) from the graph-search-network repo,
% emitting the results as Monocurl-ready data definitions plus the exploration
% traces.
%
% This file re-implements each search with instrumentation so we capture the
% ORDER in which nodes are EXPANDED (popped from the frontier). That order is
% what the Monocurl visualization animates to reveal how each algorithm
% explores the graph.
%
% Usage:
%   swipl -q -f scripts/generate_monocurl_graph_data.pl \
%       -g "run(N, K, P, Seed, Vi, Vf, OutFile)" -t halt
%
% Example (matches the params used in visualization.mcs):
%   swipl -q -f scripts/generate_monocurl_graph_data.pl \
%       -g "run(30, 5, 0.15, 31, 0, 29, 'visualization-support/graph_data.mcs')" -t halt
%
% Arguments:
%   N    : number of vertices in the small-world graph
%   K    : degree of the nearest-neighbor backbone (k nearest neighbours)
%   P    : edge rewiring probability (0..1)
%   Seed : seed for SWI-Prolog's random generator (reproducible graphs)
%   Vi   : start vertex id (must be < N, != Vf)
%   Vf   : goal vertex id (must be < N)
%   Out  : path of the generated .mcs data file
%
% Heuristics:
%   The A* and Best-First searches score candidates with a heuristic h(x) from
%   the candidate vertex to the goal. By default h'(x) = h(x) (euclidean
%   distance). Use set_heur_mode/1 before run/7 to compose the heuristic:
%     set_heur_mode(composed).    % h'(x) = h(x)          (default)
%     set_heur_mode(expr(E)).     % h'(x) = E              (arbitrary expression)
%
% HOW TO INTEGRATE WITH MONOCURL
%   Monocurl has no way to run Prolog, so this driver writes a generated .mcl
%   file. The map is named after the file: gen/graph20.mcl defines `graph20`,
%   gen/graph.mcl defines `graph`, and so on. Scenes import that file and pass
%   the map to the graph functions in common.mcl.
%
% The generated file defines a single map with the graph plus every algorithm
% trace. All keys are strings, so access uses graph["nodes"], graph["dijkstra"],
% etc. Graph-level keys:
%   "nodes"         : list of [id, x, y]
%   "edges"         : list of [a, b, dist] (the final small-world graph)
%   "knn_edges"     : [a, b, dist] edges still present from the KNN backbone
%   "rewired_edges" : [a, b, dist] edges introduced by the rewiring step
%   "n", "k", "p", "seed" : generator parameters
%   "start_id"      : search source vertex id
%   "goal_id"       : search target vertex id
%
% Each algorithm is a nested map. For example, graph["dijkstra"] contains:
%   "expand", "path", "dist", "g", "open", "pick", "relax", "pred"
% graph["astar"] contains "expand", "path", "dist", "steps", "pending";
% graph["hill"] contains "expand", "path", "dist", "status", "steps";
% graph["best"] contains "expand", "path", "dist", "steps", "pending", "pred";
% graph["bfs"] and graph["dfs"] contain "expand", "path", "steps".
%
% Every "path" is ordered start -> goal, so the scene can draw it directly as a
% Polyline. The "expand" sequences are de-duplicated (first visit wins) so the
% animation reveals the set of explored nodes instead of repeated frontier
% entries.

:- ensure_loaded('../searchs/dfs.pl').  % defines filhos/4 (and ensures graph.pl)
:- ensure_loaded('../searchs/bfs.pl').
:- use_module(library(lists)).

% ---------------------------------------------------------------------------
% ---------- instrumented Dijkstra ------------------------------------------
% Expands nodes in non-decreasing distance order. Matches the repo's
% dijkstra.pl by keeping a CLOSED set (Fechados) so every node is expanded at
% most once.
%
% In addition to the pop order, each expansion records the hidden state that
% drives Dijkstra's decisions:
%   DIJK_G      : g(expand[i]) (distance from the start)
%   DIJK_OPEN   : after expanding expand[i], the deduped open list as
%                 [id, g] pairs in ascending g order
%   DIJK_PICK   : the next node (head of that open list; -1 after the goal)
%   DIJK_RELAX  : candidate relaxations from expand[i], as
%                 [neighbour, edge_weight, candidate_g, improved (1/0)]
%   DIJK_PRED   : the predecessor edge [parent, expand[i]] ([] for the start)
% ---------------------------------------------------------------------------
dij_exp(Vf, _, [(Cur-(Vf-Path)) | _], _, ExplAcc, OpenAcc, PickAcc,
        RelaxAcc, PredAcc, GAcc, Path, Cur) :-
    reverse([Vf | ExplAcc], Order),
    reverse([[] | OpenAcc], OpenRows),
    reverse([-1 | PickAcc], PickRows),
    reverse([[] | RelaxAcc], RelaxRows),
    reverse([[] | PredAcc], PredRows),
    reverse([Cur | GAcc], Gs),
    retractall(dij_open(_)), assert(dij_open(OpenRows)),
    retractall(dij_pick(_)), assert(dij_pick(PickRows)),
    retractall(dij_relax(_)), assert(dij_relax(RelaxRows)),
    retractall(dij_pred(_)), assert(dij_pred(PredRows)),
    retractall(dij_g(_)), assert(dij_g(Gs)),
    assert_results(dij, Order, Path, Cur, reach).

dij_exp(Vf, Arestas, [(Cur-(Atual-Path)) | Resto], Fechados,
        ExplAcc, OpenAcc, PickAcc, RelaxAcc, PredAcc, GAcc,
        PathFinal, DistOut) :-
    % Candidate edges from Atual. `improved` compares the candidate g with the
    % cheapest g already queued for that node (Resto is the queue BEFORE
    % appending the new entries).
    findall([Id, P, Dn, Improved],
        (   ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, Path),
            \+ member(Id, Fechados),
            Dn is Cur + P,
            queue_best_g(Id, Resto, Best),
            (   Best == none
            ->  Improved = 1
            ;   (   Dn < Best
                ->  Improved = 1
                ;   Improved = 0
                )
            )
        ),
        RelaxRow),
    findall(Dn-(Id-[Id | Path]),
        (   ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, Path),
            \+ member(Id, Fechados),
            Dn is Cur + P
        ),
        Novos),
    append(Novos, Resto, NovaFila),
    keysort(NovaFila, FilaOrdenada),
    % Drop stale queue entries for nodes that are already closed, so the
    % visualized open list only ever contains nodes waiting to be expanded.
    exclude_closed(FilaOrdenada, [Atual | Fechados], FilaAberta),
    % Open list as deduped [Id, G] pairs (first occurrence = smallest g) plus
    % the head id (the minimum-g node Dijkstra pops next).
    pack_open_dij(FilaAberta, OpenPairs, Head),
    pred_edge(Atual, Path, PredEdge),
    % When the open list empties the goal is unreachable.  throw/1 unwinds the
    % stack deterministically (no backtracking into findall's choice points,
    % which is what made plain fail/! loop forever under SWI-Prolog's
    % last-call optimisation).
    (   FilaAberta == []
    ->  throw(unreachable)
    ;   dij_exp(Vf, Arestas, FilaAberta, [Atual | Fechados], [Atual | ExplAcc],
            [OpenPairs | OpenAcc], [Head | PickAcc], [RelaxRow | RelaxAcc],
            [PredEdge | PredAcc], [Cur | GAcc], PathFinal, DistOut)
    ).

% exclude_closed/3 : remove queue entries (G-(Id-Path)) whose Id is in Closed.
exclude_closed([], _, []).
exclude_closed([Entry | Rest], Closed, Out) :-
    Entry = _G-(Id-_),
    (   memberchk(Id, Closed)
    ->  Out = OutRest
    ;   Out = [Entry | OutRest]
    ),
    exclude_closed(Rest, Closed, OutRest).

% queue_best_g/3 : cheapest g currently queued for Id, or the atom `none`.
queue_best_g(Id, Queue, Best) :-
    findall(G, member(G-(Id-_), Queue), Gs),
    (   Gs == []
    ->  Best = none
    ;   min_list(Gs, Best)
    ).

% pred_edge/3 : predecessor edge for Atual from its stored path.
% Path is [Atual, Parent, ...], so the parent is the second element; the start
% node has no predecessor.
pred_edge(_Atual, [_], []).
pred_edge(Atual, [Atual, Parent | _], [Parent, Atual]).

% pack_open_dij: from FilaOrdenada of (G-(Id-Path)), build the deduped
% [[Id, G], ...] frontier (first occurrence = smallest g) and the head id (the
% minimum-g node Dijkstra pops next).
pack_open_dij(Fila, OpenPairs, Head) :-
    findall([Id, G], member(G-(Id-_), Fila), AllPairs),
    dedupe_pairs(AllPairs, OpenPairs),
    ( Fila = [_Gf-(Head-_) | _] -> true ; Head = -1 ).

% ---------------------------------------------------------------------------
% ---------- heuristic composition -----------------------------------------
% A* and Best-First drive their decisions with a heuristic h(x) measured from
% the candidate vertex to the goal. To run them with a composed heuristic h'(x)
% instead of the plain h(x), select a transformation with set_heur_mode/1:
%   composed     -> h'(x) = h(x)           (default; plain euclidean distance)
%   expr(E)      -> h'(x) = E                (arbitrary arithmetic expression
%                                           over the base heuristic h(x), e.g.
%                                           2 * h, h ^ 2, sqrt(h) + h)
% The composed value replaces h in f = g + h (A*) and in the open-list key
% (Best-First), so the recorded expansions and pending sets reflect the
% composed heuristic.
% ---------------------------------------------------------------------------
:- dynamic heur_mode/1.
heur_mode(composed).

set_heur_mode(M) :- retractall(heur_mode(_)), assertz(heur_mode(M)).

apply_heur(composed, H, H) :- !.

apply_heur(expr(E), H0, H) :-
    subst_expr(E, H0, EVal),
    H is EVal.

% subst_expr/3 : inside an arithmetic expression over the base heuristic (e.g.
% `2 * H0`, `H0 ^ 2`) the base appears as a plain variable; replace it with its
% bound value so the composed heuristic can be evaluated with is/2.
subst_expr(T, _V, T) :- number(T), !.
subst_expr(T, V, S) :- var(T), !, S = V.
subst_expr(T, V, S) :-
    T =.. Rest,
    [F | Args] = Rest,
    subst_args(Args, V, NewArgs),
    S =.. [F | NewArgs].

subst_args([], _, []).
subst_args([A | As], V, [SA | SS]) :-
    subst_expr(A, V, SA),
    subst_args(As, V, SS).

% heur_val/4 : effective heuristic from candidate vertex IdFrom to the goal
% Vf, composed with the currently selected heuristic transformation.
heur_val(Vf, Vertices, IdFrom, H) :-
    member(v(IdFrom, X1, Y1), Vertices),
    member(v(Vf, X2, Y2), Vertices),
    distancia(v(IdFrom, X1, Y1), v(Vf, X2, Y2), H0),
    heur_mode(M),
    apply_heur(M, H0, H).

% ---------------------------------------------------------------------------
% ---------- instrumented A* -------------------------------------------------
% f = g + h ; h is the (composed) heuristic to the goal.
% Also records, for each expanded node, the unvisited candidate neighbours that
% were scored with the heuristic, and the node A* expands next (min f = g + h).
% The expansion loop is iterative through the tail-recursive worker astar_run/9
% so that a deep frontier cannot grow the stack without bound.
% ---------------------------------------------------------------------------
astar_exp(Vf, Vertices, Arestas, Frontier, ExplAcc, StepsAcc, PendAcc, PathFinal, DistOut) :-
    astar_run(Vf, Vertices, Arestas, Frontier, ExplAcc, StepsAcc, PendAcc, PathFinal, DistOut).

astar_run(Vf, Vertices, Arestas, Frontier, ExplAcc, StepsAcc, PendAcc, PathFinal, DistOut) :-
    (   Frontier = [(_F-Cur-(Vf-Path)) | _]
    ->  reverse([Vf | ExplAcc], Order),
       reverse([[Vf, [], []] | StepsAcc], Steps),
       reverse([[] | PendAcc], Pending),
       retractall(astar_steps(_)), assert(astar_steps(Steps)),
       retractall(astar_pending(_)), assert(astar_pending(Pending)),
       assert_results(astar, Order, Path, Cur, reach), !
    ;   Frontier = [_F-Cur-(Atual-Path) | Resto]
    ->  findall(Fn-Dn-(Id-[Id | Path]),
            (   ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
                \+ member(Id, Path),
                \+ member(Id, ExplAcc),
                Dn is Cur + P,
                member(v(Id, X1, Y1), Vertices),
                member(v(Vf, X2, Y2), Vertices),
                heur_val(Vf, Vertices, Id, H),
                Fn is Cur + P + H
            ),
            Novos),
       findall(Id, member(_Fn-_Dn-(Id-_), Novos), CandIds),
       append(Novos, Resto, NovaFila),
       keysort(NovaFila, FilaOrdenada),
       pack_pending(FilaOrdenada, PendingRow),
       (   FilaOrdenada = [_FB-_DB-(NextId-_) | _]
       ->  true
       ;   NextId = -1
       ),
       astar_run(Vf, Vertices, Arestas, FilaOrdenada,
                 [Atual | ExplAcc], [[Atual, CandIds, NextId] | StepsAcc],
                 [PendingRow | PendAcc], PathFinal, DistOut)
    ;   true
    ->  true
    ).

% pack_pending/2 : turn a FilaOrdenada of (F-D-(Id-Path)) into [[Id, F], ...].
pack_pending([], []).
pack_pending([F-_D-(Id-_) | Resto], [[Id, F] | Acc]) :-
    pack_pending(Resto, Acc).



% ---------------------------------------------------------------------------
% ---------- instrumented Hill Climbing --------------------------------------
% Greedy: always steps to the closest unvisited neighbour. Records every node
% visited, its unvisited candidate neighbours (with the heuristic look), and
% the node it actually picks (min edge weight).
% ---------------------------------------------------------------------------
hc_exp(Vf, Vf, _Arestas, Visitados, ExplAcc, StepsAcc, Dist, Dist) :-
    reverse([Vf | ExplAcc], Order),
    reverse(Visitados, PathRev),
    reverse([[Vf, [], []] | StepsAcc], Steps),
    retractall(hill_steps(_)), assert(hill_steps(Steps)),
    assert_results(hill, Order, PathRev, Dist, reach), !.

hc_exp(Atual, _Vf, Arestas, Visitados, ExplAcc, StepsAcc, DistAc, _Dist) :-
    findall(P-Id,
        (   ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, Visitados)
        ),
        Candidatos),
    Candidatos = [],
    reverse([Atual | ExplAcc], Order),
    reverse(Visitados, PathRev),
    reverse([[Atual, [], []] | StepsAcc], Steps),
    retractall(hill_steps(_)), assert(hill_steps(Steps)),
    assert_results(hill, Order, PathRev, DistAc, stuck), !.

hc_exp(Atual, Vf, Arestas, Visitados, ExplAcc, StepsAcc, DistAc, Dist) :-
    findall(P-Id,
        (   ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, Visitados)
        ),
        Candidatos),
    % candidates as [neighbour, edge_weight] pairs so the scene can show how the
    % greedy step picks the minimum-weight neighbour
    findall([Id, P2], member(P2-Id, Candidatos), CandPairs),
    keysort(Candidatos, Ordenados),
    Ordenados = [(P-NovoNode) | _],
    DistNova is DistAc + P,
    hc_exp(NovoNode, Vf, Arestas, [NovoNode | Visitados],
           [Atual | ExplAcc], [[Atual, CandPairs, NovoNode] | StepsAcc],
           DistNova, Dist).

% ---------------------------------------------------------------------------
% ---------- instrumented Best-First (greedy, key = h) -----------------------
% Mirrors searchs/bestFirst.pl: the queue holds H-(Id-Path) entries, children
% come from filhos/4 (so only cycles WITHIN a path are avoided - there is no
% closed set), each child is keyed by its straight-line distance to the goal,
% new entries are appended AFTER the old ones and the queue is re-sorted with
% keysort/2 (stable, so ties keep queue order). The search stops when the head
% of the queue is the goal.
%
% Because there is no closed set, a node may be popped more than once (via
% different paths). Like the other searches, only FIRST visits become
% visualization steps, so BEST_EXPAND and every BEST_* row stay aligned.
% ---------------------------------------------------------------------------
best_exp(Vf, _, _, [_H-(Vf-CaminhoAtual) | _], ExplAcc, StepsAcc, PendAcc, PredAcc, Caminho) :-
    reverse(CaminhoAtual, Caminho),
    reverse([Vf | ExplAcc], Order),
    reverse([[Vf, [], -1] | StepsAcc], Steps0),
    reverse([[] | PendAcc], Pending),
    pred_edge(Vf, CaminhoAtual, PredEdge),
    reverse([PredEdge | PredAcc], Preds),
    dedupe(Order, OrderD),
    best_set_next(Steps0, OrderD, Steps),
    retractall(best_steps(_)), assert(best_steps(Steps)),
    retractall(best_pending(_)), assert(best_pending(Pending)),
    retractall(best_pred(_)), assert(best_pred(Preds)),
    assert_results(best, Order, Caminho, 0, reach), !.

best_exp(Vf, Vertices, Arestas, [_H-(Atual-CaminhoAtual) | DemaisCaminhos],
         ExplAcc, StepsAcc, PendAcc, PredAcc, Caminho) :-
    filhos(Atual, Arestas, CaminhoAtual, L),
    % same findall / append / keysort as adicionar_caminhos_best/6
    findall(D-(X-[X | CaminhoAtual]),
        (   member(X, L),
            member(v(X, X1, Y1), Vertices),
            member(v(Vf, X2, Y2), Vertices),
            heur_val(Vf, Vertices, X, D)
        ),
        CaminhosNovos),
    append(DemaisCaminhos, CaminhosNovos, S),
    keysort(S, CaminhosAtualizados),
    (   memberchk(Atual, ExplAcc)
    ->  NextSteps = StepsAcc, NextPend = PendAcc, NextPred = PredAcc
    ;   findall([Id, Dh], member(Dh-(Id-_), CaminhosNovos), CandPairs),
        % open list after this expansion: [Id, h] pairs, deduped by id, ids
        % already expanded removed (they would only be re-expanded duplicates)
        exclude_open(CaminhosAtualizados, [Atual | ExplAcc], Open0),
        dedupe_pairs(Open0, OpenPairs),
        pred_edge(Atual, CaminhoAtual, PredEdge),
        NextSteps = [[Atual, CandPairs, -1] | StepsAcc],
        NextPend = [OpenPairs | PendAcc],
        NextPred = [PredEdge | PredAcc]
    ),
    best_exp(Vf, Vertices, Arestas, CaminhosAtualizados,
             [Atual | ExplAcc], NextSteps, NextPend, NextPred, Caminho).

% exclude_open/3 : queue (H-(Id-_)) -> [[Id, H], ...] without ids in Closed.
exclude_open([], _, []).
exclude_open([H-(Id-_) | Rest], Closed, Out) :-
    (   memberchk(Id, Closed)
    ->  Out = OutRest
    ;   Out = [[Id, H] | OutRest]
    ),
    exclude_open(Rest, Closed, OutRest).

% best_set_next/3 : fill each step's `chosen` with the next first-visit node.
best_set_next([], _, []).
best_set_next([[Id, Cands, _] | Rows], [_ | Rest], [[Id, Cands, Next] | Out]) :-
    (   Rest = [Next | _] -> true ; Next = -1 ),
    best_set_next(Rows, Rest, Out).

% path_dist/3 : sum of the edge weights along a path (forward order).
path_dist([_], _, 0) :- !.
path_dist([A, B | Rest], Arestas, D) :-
    (   member(e(A, B, P), Arestas) ; member(e(B, A, P), Arestas) ), !,
    path_dist([B | Rest], Arestas, D0),
    D is D0 + P.

% ---------------------------------------------------------------------------
% ---------- instrumented BFS / DFS ------------------------------------------
% The repo's bfs/dfs take (Vi, Vf, Vertices, Arestas) and search a fixed graph.
% We mirror their *busca* logic (same child generation via filhos/4) while
% recording the order in which nodes are popped, the live queue/stack, the
% parent edges that form the search tree, and the solution path.
%
% Steps are:
%   BFS_STEPS : [current, queue_after, [[parent, child], ...], depth]
%   DFS_STEPS : [current, current_path, stack_heads, [[parent, child], ...],
%                depth, backtracked?]  (1/0)
% Queue/stack ids are de-duped for display, while the underlying search still
% behaves exactly like the repo's implementation.
% ---------------------------------------------------------------------------
bfs_exp(Vf, _, [Vf-CaminhoAtual | _], ExplAcc, StepsAcc, Caminho) :-
    reverse([Vf | ExplAcc], Order),
    reverse(CaminhoAtual, Caminho),
    length(CaminhoAtual, Len), Depth is Len - 1,
    reverse([[Vf, [], [], Depth] | StepsAcc], Steps),
    retractall(bfs_steps(_)), assert(bfs_steps(Steps)),
    assert_results(bfs, Order, Caminho, 0, reach), !.

bfs_exp(Vf, Arestas, [Atual-CaminhoAtual | Resto], ExplAcc, StepsAcc, Caminho) :-
    filhos(Atual, Arestas, CaminhoAtual, L),
    subtract(L, ExplAcc, Lnovos),          % visited set = ExplAcc (expanded so far)
    findall(X-[X | CaminhoAtual], member(X, Lnovos), CaminhosNovos),
    append(Resto, CaminhosNovos, Atualizados),
    queue_ids(Atualizados, QueueRaw0),
    exclude_ids(QueueRaw0, [Atual | ExplAcc], QueueActive),
    dedupe(QueueActive, QueueIds),
    queue_ids(Resto, RestoIds),
    exclude_ids(Lnovos, RestoIds, NewIds),
    findall([Atual, Id], member(Id, NewIds), ParentEdges),
    length(CaminhoAtual, Len), Depth is Len - 1,
    % Only first visits become visualization steps; re-expansions through
    % duplicate queue entries are an implementation detail, not a decision.
    (   memberchk(Atual, ExplAcc)
    ->  NextSteps = StepsAcc
    ;   NextSteps = [[Atual, QueueIds, ParentEdges, Depth] | StepsAcc]
    ),
    bfs_exp(Vf, Arestas, Atualizados, [Atual | ExplAcc], NextSteps, Caminho).

dfs_exp(Vf, _, [Vf-CaminhoAtual | _], ExplAcc, StepsAcc, PrevPath, Caminho) :-
    reverse([Vf | ExplAcc], Order),
    reverse(CaminhoAtual, Caminho),
    length(CaminhoAtual, Len), Depth is Len - 1,
    backtracked(PrevPath, Caminho, Backtracked),
    reverse([[Vf, Caminho, [], [], Depth, Backtracked] | StepsAcc], Steps),
    retractall(dfs_steps(_)), assert(dfs_steps(Steps)),
    assert_results(dfs, Order, Caminho, 0, reach), !.

dfs_exp(Vf, Arestas, [Atual-CaminhoAtual | Resto], ExplAcc, StepsAcc, PrevPath, Caminho) :-
    filhos(Atual, Arestas, CaminhoAtual, L),
    subtract(L, ExplAcc, Lnovos),
    findall(X-[X | CaminhoAtual], member(X, Lnovos), CaminhosNovos),
    append(CaminhosNovos, Resto, Atualizados),
    reverse(CaminhoAtual, CurrentPath),
    length(CaminhoAtual, Len), Depth is Len - 1,
    backtracked(PrevPath, CurrentPath, Backtracked),
    queue_ids(Atualizados, HeadsRaw0),
    exclude_ids(HeadsRaw0, [Atual | ExplAcc], StackActive),
    dedupe(StackActive, StackHeads),
    queue_ids(Resto, RestoIds),
    exclude_ids(Lnovos, RestoIds, NewIds),
    findall([Atual, Id], member(Id, NewIds), ParentEdges),
    (   memberchk(Atual, ExplAcc)
    ->  NextSteps = StepsAcc
    ;   NextSteps = [[Atual, CurrentPath, StackHeads, ParentEdges, Depth,
                      Backtracked] | StepsAcc]
    ),
    dfs_exp(Vf, Arestas, Atualizados, [Atual | ExplAcc],
            NextSteps, CurrentPath, Caminho).

% queue_ids/2 : node ids at the head of each queued partial path.
queue_ids([], []).
queue_ids([Id-_ | Rest], [Id | Ids]) :- queue_ids(Rest, Ids).

% exclude_ids/3 : remove elements of Excl from List (stable order).
exclude_ids([], _, []).
exclude_ids([X | Xs], Excl, Out) :-
    (   memberchk(X, Excl)
    ->  Out = OutRest
    ;   Out = [X | OutRest]
    ),
    exclude_ids(Xs, Excl, OutRest).

% backtracked/3 : true when CurrentPath is not an extension of PrevPath, i.e.
% DFS moved to a sibling branch instead of descending.
backtracked([], _, 0) :- !.
backtracked(Prev, Current, 1) :- \+ is_prefix(Prev, Current), !.
backtracked(_, _, 0).

is_prefix([], _).
is_prefix([X | Xs], [X | Ys]) :- is_prefix(Xs, Ys).

% ---------------------------------------------------------------------------
% ---------- result storage ---------------------------------------------------
% ---------------------------------------------------------------------------
:- dynamic dij/4, astar/4, hill/4, astar_steps/1, hill_steps/1,
   astar_pending/1, dij_open/1, dij_pick/1, dij_relax/1, dij_pred/1, dij_g/1,
   bfs/4, dfs/4, bfs_steps/1, dfs_steps/1,
   best/4, best_steps/1, best_pending/1, best_pred/1.

assert_results(dij, Order, Path, D, Status) :-
    retractall(dij(_, _, _, _)), assert(dij(Order, Path, D, Status)).
assert_results(astar, Order, Path, D, Status) :-
    retractall(astar(_, _, _, _)), assert(astar(Order, Path, D, Status)).
assert_results(hill, Order, Path, D, Status) :-
    retractall(hill(_, _, _, _)), assert(hill(Order, Path, D, Status)).
assert_results(best, Order, Path, D, Status) :-
    retractall(best(_, _, _, _)), assert(best(Order, Path, D, Status)).
assert_results(bfs, Order, Path, D, Status) :-
    retractall(bfs(_, _, _, _)), assert(bfs(Order, Path, D, Status)).
assert_results(dfs, Order, Path, D, Status) :-
    retractall(dfs(_, _, _, _)), assert(dfs(Order, Path, D, Status)).

% ---------------------------------------------------------------------------
% ---------- Monocurl list output ---------------------------------------------
% Convert Prolog terms (v/3, e/3, plain numbers) into Monocurl list literals so
% the generated file can be pasted straight into a .mcs scene.
%   v(Id, X, Y)  ->  [Id, X, Y]
%   e(U, V, D)   ->  [U, V, D]
% ---------------------------------------------------------------------------
emit_list(S, List) :-
    write_string(S, '['),
    emit_items(S, List),
    write_string(S, ']').

emit_items(_, []).
emit_items(S, [X | Xs]) :-
    emit_item(S, X),
    ( Xs == [] -> true ; ( write_string(S, ', '), emit_items(S, Xs) ) ).

emit_item(S, v(A, B, C)) :- emit_list(S, [A, B, C]).
emit_item(S, e(A, B, C)) :- emit_list(S, [A, B, C]).
emit_item(S, X) :- write_string(S, X).

% ---------------------------------------------------------------------------
% ---------- Simple Enhanced Nudge (Original Algorithm, Better Params) -------
% ---------------------------------------------------------------------------
% The original nudge but with: more iterations (30), larger coordinate space
% (now 0..50 from graph.pl), stronger repulsion (dist < 2.5), 
% and better factor. Runs fast and improves layout.
% ---------------------------------------------------------------------------

% Main entry point: force_directed_layout(+Vertices, +Edges, -LaidOutVertices)
force_directed_layout(Vertices, _Edges, LaidOut) :-
    nudge_vertices_simple(Vertices, LaidOut, 60).

% Simple repulsion-only nudge (original behavior, improved parameters)
nudge_vertices_simple(Vertices, Nudged) :-
    nudge_vertices_simple(Vertices, Nudged, 30).

nudge_vertices_simple(Vertices, Vertices, 0) :- !.
nudge_vertices_simple(In, Out, Iter) :-
    Iter1 is Iter - 1,
    nudge_all_pairs_simple(In, Once),
    (   Once = In
    ->  Out = Once
    ;   nudge_vertices_simple(Once, Out2, Iter1),
        Out = Out2
    ).

nudge_all_pairs_simple([], []).
nudge_all_pairs_simple([V | Rest], [V1 | Rest1]) :-
    nudge_away_simple(V, Rest, V1),
    nudge_all_pairs_simple(Rest, Rest1).

nudge_away_simple(V, [], V).
nudge_away_simple(v(I, X0, Y0), [v(_, X2, Y2) | Others], v(I, XN, YN)) :-
    Dist is sqrt((X0 - X2)*(X0 - X2) + (Y0 - Y2)*(Y0 - Y2)),
    (   Dist < 4.0, Dist > 0.01      % Increased threshold from 3.0 to 4.0
    ->  Factor is (4.0 - Dist) / 3,  % Stronger push: was (3.0-Dist)/4, now (4.0-Dist)/3
        DX is (X0 - X2) * Factor,
        DY is (Y0 - Y2) * Factor,
        X1 is X0 + DX,
        Y1 is Y0 + DY,
        % Clamp to bounds (0..50 with small margin)
        Margin = 0.5,
        FrameMax = 49.5,
        X1c is max(Margin, min(FrameMax, X1)),
        Y1c is max(Margin, min(FrameMax, Y1)),
        nudge_away_simple(v(I, X1c, Y1c), Others, v(I, XN, YN))
    ;   nudge_away_simple(v(I, X0, Y0), Others, v(I, XN, YN))
    ).

% ---------------------------------------------------------------------------
% ---------- Backward-compatible nudge wrapper -------------------------------
% ---------------------------------------------------------------------------
% Kept for compatibility; now delegates to force-directed layout when edges
% are available, otherwise falls back to simple repulsion nudge.

% nudge_vertices/2 : enhanced version using force-directed layout if edges provided
nudge_vertices(Vertices, Nudged) :-
    % Called from run/7 without edges - use simple nudge for backward compat
    nudge_vertices_simple(Vertices, Nudged, 40).

% nudge_vertices/3 : enhanced version with edges for force-directed layout
nudge_vertices(Vertices, Edges, Nudged) :-
    force_directed_layout(Vertices, Edges, Nudged).

% ---------------------------------------------------------------------------
% ---------- main runner ------------------------------------------------------
% ---------------------------------------------------------------------------
run(N, K, P, Seed, Vi, Vf, Out) :-
    set_graph_seed(Seed),          % pick reproducible seed for gera_rede_pequeno_mundo
    gera_vertices(N, VerticesRaw),
    % Build initial graph topology (KNN + rewiring)
    gera_arestas(VerticesRaw, K, KNN),
    reconecta_arestas(KNN, VerticesRaw, P, _ArestasRaw),
    % Apply enhanced nudge for better layout
    nudge_vertices_simple(VerticesRaw, Vertices),
    % Rebuild edges with NEW positions (weights = Euclidean distance in laid-out space)
    gera_arestas(Vertices, K, KNN2),
    % Track KNN edges after nudge (same topology, new positions)
    sort(KNN2, KNNS2),
    reconecta_arestas(KNN2, Vertices, P, Arestas),
    sort(Arestas, ArestasS),
    sort(Vertices, VerticesS),
    % Determine which edges were rewired (in final graph but not in KNN)
    find_rewired_edges(KNNS2, ArestasS, RewiredEdges),
    % KNN edges that remain in final graph
    find_knn_edges(KNNS2, ArestasS, KNNEdgesFinal),
    % Run the instrumented searches.  On seeds where the goal is unreachable a
    % search would spin forever under SWI-Prolog's tail-call handling of
    % findall/5, so each search throws unreachable instead; catch it and
    % degrade to a graph-only output file.
    catch(
      ( dij_exp(Vf, ArestasS, [(0-(Vi-[Vi]))], [Vi], [], [], [], [], [], [], _PD, _DD),
        astar_exp(Vf, VerticesS, ArestasS, [(0-0-(Vi-[Vi]))], [], [], [], _PA, _DA),
        hc_exp(Vi, Vf, ArestasS, [Vi], [], [], 0, _PH),
        bfs_exp(Vf, ArestasS, [Vi-[Vi]], [], [], _PB),
        dfs_exp(Vf, ArestasS, [Vi-[Vi]], [], [], [], _Pdfs),
        best_exp(Vf, VerticesS, ArestasS, [0-(Vi-[Vi])], [], [], [], [], _Pbest),
        dij(OrderD0, PathD, DistD, _SD),
        astar(OrderA0, PathA, DistA, _SA),
        hill(OrderH0, PathH, DistH, StatusH),
        bfs(OrderB, PathB, _, _),
        dfs(OrderDfs, PathDfs, _, _),
        best(OrderBest0, PathBest, _, _),
        best_steps(BestSteps),
        best_pending(BestPending),
        best_pred(BestPred),
        path_dist(PathBest, ArestasS, DistBest),
        astar_steps(AstarSteps),
        astar_pending(AstarPending),
        dij_open(DijOpen),
        dij_pick(DijPick),
        dij_relax(DijRelax),
        dij_pred(DijPred),
        dij_g(DijG),
        hill_steps(HillSteps),
        bfs_steps(BfsSteps),
        dfs_steps(DfsSteps)
      ),
      unreachable,
      ( % Goal unreachable: empty search results, graph is still emitted.
        format(user_error,
               'WARNING: goal ~w unreachable from ~w in N=~w K=~w P=~w seed=~w graph; search results empty~n',
               [Vf, Vi, N, K, P, Seed]),
        OrderD0 = [], PathD = [], DistD = 0,
        OrderA0 = [], PathA = [], DistA = 0,
        OrderH0 = [], PathH = [], DistH = 0, StatusH = "stuck",
        OrderBest0 = [], PathBest = [], DistBest = 0,
        OrderB = [], PathB = [],
        OrderDfs = [], PathDfs = [],
        DijOpen = [], DijPick = [], DijRelax = [], DijPred = [], DijG = [],
        AstarSteps = [], AstarPending = [],
        HillSteps = [], BfsSteps = [], DfsSteps = [],
        BestSteps = [], BestPending = [], BestPred = []
      )
    ),
    % The naive frontier-based searches may expand the same node more than
    % once (via different paths). For visualization we de-duplicate the
    % expansion sequence (first-visit order) so the animation shows the set of
    % nodes each algorithm explores rather than thousands of repeated visits.
    dedupe(OrderD0, OrderD),
    dedupe(OrderA0, OrderA),
    dedupe(OrderH0, OrderH),
    dedupe(OrderB, OrderBd),
    dedupe(OrderDfs, OrderDfsd),
    dedupe(OrderBest0, OrderBest),
    % Dijkstra and A* accumulate their paths goal-first; reverse them so every
    % *_PATH is in forward order (start -> goal) for drawing. BFS/DFS already
    % reverse in their base case, and Hill Climbing records its path forward.
    reverse(PathD, PathDf),
    reverse(PathA, PathAf),
    write_data(Out, N, K, P, Seed, Vi, Vf,
               VerticesS, ArestasS, KNNEdgesFinal, RewiredEdges,
               OrderD, PathDf, DistD, DijOpen, DijPick, DijRelax, DijPred, DijG,
               OrderA, PathAf, DistA, AstarSteps, AstarPending,
               OrderH, PathH, DistH, StatusH, HillSteps,
               OrderBest, PathBest, DistBest, BestSteps, BestPending, BestPred,
               OrderBd, PathB, BfsSteps, OrderDfsd, PathDfs, DfsSteps).

run(N, K, P, Seed, Vi, Vf) :-
    atomic_concat(['graph_data_n', N, '_k', K, '_p', P, '_s', Seed, '.mcs'], Out),
    run(N, K, P, Seed, Vi, Vf, Out).

write_data(Out, N, K, P, Seed, Vi, Vf, Vertices, Arestas, KNNEdgesFinal, RewiredEdges,
           OrderD, PathD, DistD, DijOpen, DijPick, DijRelax, DijPred, DijG,
           OrderA, PathA, DistA, AstarSteps, AstarPending,
           OrderH, PathH, DistH, StatusH, HillSteps,
           OrderBest, PathBest, DistBest, BestSteps, BestPending, BestPred,
           OrderB, PathB, BfsSteps, OrderDfs, PathDfs, DfsSteps) :-
    file_base_name(Out, FileName),
    file_name_extension(GraphName, mcl, FileName),
    open(Out, write, S),
    format(S, '~n# AUTO-GENERATED by scripts/generate_monocurl_graph_data.pl~n', []),
    format(S, '# Do not edit by hand - rerun the generator to refresh.~n', []),
    format(S, '# Small-World graph: N=~w  K=~w  P=~w  seed=~w~n', [N, K, P, Seed]),
    format(S, '# Start=~w  Goal=~w~n~n', [Vi, Vf]),
    format(S, '# One map with the graph and every algorithm trace. Keys are strings.~n~n', []),

    format(S, 'let ~w = [~n', [GraphName]),
    write_fields(S, '    ',
        [ list("nodes", Vertices),
          list("edges", Arestas),
          list("knn_edges", KNNEdgesFinal),
          list("rewired_edges", RewiredEdges),
          atom("n", N),
          atom("k", K),
          atom("p", P),
          atom("seed", Seed),
          atom("start_id", Vi),
          atom("goal_id", Vf),
          map("dijkstra",
              [ list("expand", OrderD),
                list("path", PathD),
                atom("dist", DistD),
                list("g", DijG),
                list("open", DijOpen),
                list("pick", DijPick),
                list("relax", DijRelax),
                list("pred", DijPred)
              ]),
          map("astar",
              [ list("expand", OrderA),
                list("path", PathA),
                atom("dist", DistA),
                list("steps", AstarSteps),
                list("pending", AstarPending)
              ]),
          map("hill",
              [ list("expand", OrderH),
                list("path", PathH),
                atom("dist", DistH),
                string("status", StatusH),
                list("steps", HillSteps)
              ]),
          map("best",
              [ list("expand", OrderBest),
                list("path", PathBest),
                atom("dist", DistBest),
                list("steps", BestSteps),
                list("pending", BestPending),
                list("pred", BestPred)
              ]),
          map("bfs",
              [ list("expand", OrderB),
                list("path", PathB),
                list("steps", BfsSteps)
              ]),
          map("dfs",
              [ list("expand", OrderDfs),
                list("path", PathDfs),
                list("steps", DfsSteps)
              ])
        ]),
    format(S, ']~n', []),
    close(S).

write_fields(S, Indent, [Field]) :-
    !,
    write_field(S, Indent, Field, last).
write_fields(S, Indent, [Field | Rest]) :-
    write_field(S, Indent, Field, comma),
    write_fields(S, Indent, Rest).

write_field(S, Indent, list(Key, List), End) :-
    format(S, '~w"~w" -> ', [Indent, Key]),
    emit_list(S, List),
    write_end(S, End).
write_field(S, Indent, atom(Key, Value), End) :-
    format(S, '~w"~w" -> ~w', [Indent, Key, Value]),
    write_end(S, End).
write_field(S, Indent, string(Key, Value), End) :-
    format(S, '~w"~w" -> "~w"', [Indent, Key, Value]),
    write_end(S, End).
write_field(S, Indent, map(Key, Fields), End) :-
    format(S, '~w"~w" -> [~n', [Indent, Key]),
    atom_concat(Indent, '    ', SubIndent),
    write_fields(S, SubIndent, Fields),
    format(S, '~w]', [Indent]),
    write_end(S, End).

write_end(S, last) :- format(S, '~n', []).
write_end(S, comma) :- format(S, ',~n', []).

% Prolog shim: write_string/2 is just write/2 to avoid ambiguity warnings.
write_string(S, X) :- write(S, X).

% dedupe/2 - stable de-duplication preserving first-occurrence order.
dedupe(List, Dedup) :- dedupe_(List, [], Dedup).
dedupe_([], Acc, Dedup) :- reverse(Acc, Dedup).
dedupe_([X | Xs], Acc, Out) :-
    (   memberchk(X, Acc) -> dedupe_(Xs, Acc, Out)
    ;   dedupe_(Xs, [X | Acc], Out)
    ).

% dedupe_pairs/2 - stable de-duplication of [Id, Value] pairs by Id, keeping
% the first occurrence (the smallest value when the input is sorted by value).
dedupe_pairs(Pairs, Dedup) :- dedupe_pairs_(Pairs, [], [], Dedup).
dedupe_pairs_([], _, Acc, Dedup) :- reverse(Acc, Dedup).
dedupe_pairs_([[Id, V] | Rest], Seen, Acc, Out) :-
    (   memberchk(Id, Seen)
    ->  dedupe_pairs_(Rest, Seen, Acc, Out)
    ;   dedupe_pairs_(Rest, [Id | Seen], [[Id, V] | Acc], Out)
    ).

% =============================================================================
% ---------- Edge classification for SW network animation ----------------------
% =============================================================================

% Find edges that were rewired (present in final graph but not in KNN)
% Edges are compared by their endpoints (U, V), ignoring weights since
% weights change after the nudge.
find_rewired_edges(KNN, Final, Rewired) :-
    findall(E,
        ( member(E, Final),
          E = e(U, V, _),
          \+ member(e(U, V, _), KNN)
        ),
        Rewired).

% Find KNN edges that remain in the final graph
find_knn_edges(KNN, Final, KNNFinal) :-
    findall(E,
        ( member(E, KNN),
          E = e(U, V, _),
          member(e(U, V, _), Final)
        ),
        KNNFinal).
