% generate_data.pl
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
%   swipl -q -f visualization-support/generate_data.pl \
%       -g "run(N, K, P, Seed, Vi, Vf, OutFile)" -t halt
%
% Example (matches the params used in visualization.mcs):
%   swipl -q -f visualization-support/generate_data.pl \
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
% HOW TO INTEGRATE WITH MONOCURL
%   Monocurl has no way to run Prolog, so this driver produces plain Monocurl
%   `let` declarations (as .mcs text) that you copy-paste into the scene file.
%   Concretely:
%     1. Run the driver to (re)generate a graph_data.mcs file.
%     2. Open visualization.mcs and replace everything between the
%        "<graph-data>" and "# <graph-data>" markers with the generated content
%        (copy it verbatim; the generator also emits the explanatory comments).
%     3. If you changed N / Start / Goal, also update the scene constants
%        (GRAPH_LOCAL_CENTER, START_ID, GOAL_ID) and the intro/comparison text
%        that mentions the graph size or the node ids.
%   The generator emits Prolog terms v(Id,X,Y) / e(U,V,D) as Monocurl lists
%   [Id,X,Y] / [U,V,D] so they can be consumed directly by the scene.
%
% The generated file defines the following Monocurl names (copy it into the
% scene file in place of the <graph-data> marker):
%   GRAPH_POINTS  : list of [id, x, y]
%   GRAPH_EDGES   : list of [a, b, dist]
%   DIJK_EXPAND   : ordered node ids expanded by Dijkstra
%   DIJK_PATH     : node ids on the Dijkstra path
%   DIJK_DIST     : total distance of that path
%   DIJK_G        : per step, the settled distance g(expand[i])
%   DIJK_OPEN     : per step, after expanding expand[i], the open list as
%                   [[id, g], ...] (deduped, ascending g)
%   DIJK_PICK     : per step, the node Dijkstra expands next (min g; -1 at end)
%   DIJK_RELAX    : per step, [neighbour, edge_weight, candidate_g, improved (1/0)]
%   DIJK_PRED     : per step, the predecessor edge [parent, child] ([] at start)
%   ASTAR_EXPAND  : ordered node ids expanded by A*
%   ASTAR_PATH    : node ids on the A* path
%   ASTAR_DIST    : total distance of that path
%   ASTAR_STEPS   : per step [current, [candidate neighbours], chosen]
%   ASTAR_PENDING : per step, after expanding expand[i], the full A* open list
%                   as [[id, f], ...] in f order
%   HILL_EXPAND   : ordered node ids visited by Hill Climbing
%   HILL_PATH     : node ids on the Hill Climbing path
%   HILL_DIST     : total distance of that path
%   HILL_STATUS   : 'reach' or 'stuck' (Hill Climbing may fail on a dead end)
%   HILL_STEPS    : per step [current, [[candidate, edge_weight], ...], chosen]
%                   (candidates in graph-edge order; chosen = minimum weight)
%   BEST_EXPAND   : first-visit order of nodes expanded by Best-First
%   BEST_PATH     : node ids on the Best-First path
%   BEST_DIST     : total distance of that path
%   BEST_STEPS    : per step [current, [[candidate, h], ...], next] - the children
%                   pushed by that expansion (h = straight-line distance to the
%                   goal, which is the queue key) and the next first-visit node
%                   (-1 on the last row)
%   BEST_PENDING  : per step, after expanding expand[i], the open list as
%                   [[id, h], ...] in queue order (deduped by id, nodes that were
%                   already expanded removed)
%   BEST_PRED     : per step, the predecessor edge [parent, child] ([] for the start)
%   BFS_EXPAND    : first-visit order of nodes expanded by BFS
%   BFS_PATH      : node ids on the BFS path
%   BFS_STEPS     : per step [current, queue_after, [[parent, child], ...], depth]
%   DFS_EXPAND    : first-visit order of nodes expanded by DFS
%   DFS_PATH      : node ids on the DFS path
%   DFS_STEPS     : per step [current, current_path, stack_heads,
%                   [[parent, child], ...], depth, backtracked?]  (1/0)
%
% Every *_PATH is ordered start -> goal, so the scene can draw it directly as a
% Polyline. The *_EXPAND sequences are de-duplicated (first visit wins) so the
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
    assert_results(dij, Order, Path, Cur, reach), !.

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
    dij_exp(Vf, Arestas, FilaAberta, [Atual | Fechados], [Atual | ExplAcc],
            [OpenPairs | OpenAcc], [Head | PickAcc], [RelaxRow | RelaxAcc],
            [PredEdge | PredAcc], [Cur | GAcc], PathFinal, DistOut).

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
% ---------- instrumented A* -------------------------------------------------
% f = g + h ; h is the euclidean distance to the goal.
% Also records, for each expanded node, the unvisited candidate neighbors that
% were scored with the heuristic, and the node A* expands next (min f = g + h).
% ---------------------------------------------------------------------------
astar_exp(Vf, _, _Arestas, [(_F-Cur-(Vf-Path)) | _], ExplAcc, StepsAcc, PendAcc, Path, Cur) :-
    reverse([Vf | ExplAcc], Order),
    reverse([[Vf, [], []] | StepsAcc], Steps),
    reverse([[] | PendAcc], Pending),
    retractall(astar_steps(_)), assert(astar_steps(Steps)),
    retractall(astar_pending(_)), assert(astar_pending(Pending)),
    assert_results(astar, Order, Path, Cur, reach), !.

astar_exp(Vf, Vertices, Arestas, [(_F-Cur-(Atual-Path)) | Resto], ExplAcc, StepsAcc, PendAcc, PathFinal, DistOut) :-
    findall(Fn-Dn-(Id-[Id | Path]),
        (   ( member(e(Atual, Id, P), Arestas) ; member(e(Id, Atual, P), Arestas) ),
            \+ member(Id, Path),
            Dn is Cur + P,
            member(v(Id, X1, Y1), Vertices),
            member(v(Vf, X2, Y2), Vertices),
            distancia(v(Id, X1, Y1), v(Vf, X2, Y2), H),
            Fn is Cur + P + H
        ),
        Novos),
    findall(Id, member(_Fn-_Dn-(Id-_), Novos), CandIds),
    append(Novos, Resto, NovaFila),
    keysort(NovaFila, FilaOrdenada),
    % Snapshot of the full pending (priority) queue: [Id, f] pairs in order.
    pack_pending(FilaOrdenada, PendingRow),
    (   FilaOrdenada = [_FB-_DB-(NextId-_) | _]
    ->  true
    ;   NextId = -1
    ),
    astar_exp(Vf, Vertices, Arestas, FilaOrdenada,
              [Atual | ExplAcc], [[Atual, CandIds, NextId] | StepsAcc],
              [PendingRow | PendAcc],
              PathFinal, DistOut).

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
            distancia(v(X, X1, Y1), v(Vf, X2, Y2), D)
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
% ---------- nudge step: afasta vértices muito próximos -----------------------
% ---------------------------------------------------------------------------

% nudge_vertices/2 : afasta pares de vértices que estão mais próximos que 1.0
% unidade, repetindo até não haver mais movimento ou atingir o limite de iterações.
nudge_vertices(Vertices, Nudged) :-
    nudge_vertices(Vertices, Nudged, 8).

nudge_vertices(Vertices, Vertices, 0) :- !.
nudge_vertices(In, Out, Iter) :-
    Iter1 is Iter - 1,
    nudge_all_pairs(In, Once),
    (   Once = In
    ->  Out = Once
    ;   nudge_vertices(Once, Out2, Iter1),
        Out = Out2
    ).

% nudge_all_pairs/2 : desloca cada vértice para longe dos vizinhos muito próximos.
nudge_all_pairs([], []).
nudge_all_pairs([V | Rest], [V1 | Rest1]) :-
    nudge_away(V, Rest, V1),
    nudge_all_pairs(Rest, Rest1).

% nudge_away/3 : desloca V para longe de cada vértice em Others que esteja muito perto.
nudge_away(_, [], V) :- V = V.
nudge_away(v(I, X0, Y0), [v(_, X2, Y2) | Others], v(I, XN, YN)) :-
    Dist is sqrt((X0 - X2)^2 + (Y0 - Y2)^2),
    (   Dist < 1.0, Dist > 0.01
    ->  Factor is (1.0 - Dist) / 4,
        DX is (X0 - X2) * Factor,
        DY is (Y0 - Y2) * Factor,
        X1 is X0 + DX,
        Y1 is Y0 + DY,
        nudge_away(v(I, X1, Y1), Others, v(I, XN, YN))
    ;   nudge_away(v(I, X0, Y0), Others, v(I, XN, YN))
    ).

% ---------------------------------------------------------------------------
% ---------- main runner ------------------------------------------------------
% ---------------------------------------------------------------------------
run(N, K, P, Seed, Vi, Vf, Out) :-
    set_graph_seed(Seed),          % pick reproducible seed for gera_rede_pequeno_mundo
    gera_vertices(N, VerticesRaw),
    % Afasta vértices que ficaram muito próximos, reduzindo a aglomeração visual.
    nudge_vertices(VerticesRaw, Vertices),
    gera_arestas(Vertices, K, KNN),
    reconecta_arestas(KNN, Vertices, P, Arestas),
    sort(Arestas, ArestasS),
    sort(Vertices, VerticesS),
    dij_exp(Vf, ArestasS, [(0-(Vi-[Vi]))], [Vi], [], [], [], [], [], [],
            _PD, _DD),
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
    dfs_steps(DfsSteps),
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
               VerticesS, ArestasS,
               OrderD, PathDf, DistD, DijOpen, DijPick, DijRelax, DijPred, DijG,
               OrderA, PathAf, DistA, AstarSteps, AstarPending,
               OrderH, PathH, DistH, StatusH, HillSteps,
               OrderBest, PathBest, DistBest, BestSteps, BestPending, BestPred,
               OrderBd, PathB, BfsSteps, OrderDfsd, PathDfs, DfsSteps).

run(N, K, P, Seed, Vi, Vf) :-
    atomic_concat(['graph_data_n', N, '_k', K, '_p', P, '_s', Seed, '.mcs'], Out),
    run(N, K, P, Seed, Vi, Vf, Out).

write_data(Out, N, K, P, Seed, Vi, Vf, Vertices, Arestas,
           OrderD, PathD, DistD, DijOpen, DijPick, DijRelax, DijPred, DijG,
           OrderA, PathA, DistA, AstarSteps, AstarPending,
           OrderH, PathH, DistH, StatusH, HillSteps,
           OrderBest, PathBest, DistBest, BestSteps, BestPending, BestPred,
           OrderB, PathB, BfsSteps, OrderDfs, PathDfs, DfsSteps) :-
    open(Out, write, S),
    format(S, '~n# AUTO-GENERATED by visualization-support/generate_data.pl~n', []),
    format(S, '# Do not edit by hand - rerun the generator to refresh.~n', []),
    format(S, '# Small-World graph: N=~w  K=~w  P=~w  seed=~w~n', [N, K, P, Seed]),
    format(S, '# Start=~w  Goal=~w~n~n', [Vi, Vf]),

    format(S, 'let GRAPH_POINTS = ', []), emit_list(S, Vertices), format(S, '~n~n', []),
    format(S, 'let GRAPH_EDGES = ', []), emit_list(S, Arestas), format(S, '~n~n', []),

    write_alg(S, 'DIJK_EXPAND', OrderD),
    write_alg(S, 'DIJK_PATH', PathD),
    format(S, 'let DIJK_DIST = ~w~n~n', [DistD]),
    format(S, '# DIJK_G: g(expand[i]), the settled distance of each expanded node.~n', []),
    write_alg(S, 'DIJK_G', DijG),
    format(S, '# DIJK_OPEN / DIJK_PICK: after expanding expand[i], the deduped open list as~n', []),
    format(S, '# [node_id, g] pairs in ascending g order, and the node expanded next~n', []),
    format(S, '# (the minimum-g head; -1 on the final row).~n', []),
    write_alg(S, 'DIJK_OPEN', DijOpen),
    write_alg(S, 'DIJK_PICK', DijPick),
    format(S, '# DIJK_RELAX: per step, [neighbour, edge_weight, candidate_g, improved (1/0)]~n', []),
    format(S, '# for every edge considered from the current node.~n', []),
    write_alg(S, 'DIJK_RELAX', DijRelax),
    format(S, '# DIJK_PRED: per step, the predecessor edge [parent, child] ([] for the start).~n', []),
    write_alg(S, 'DIJK_PRED', DijPred),
    format(S, '~n', []),

    write_alg(S, 'ASTAR_EXPAND', OrderA),
    write_alg(S, 'ASTAR_PATH', PathA),
    format(S, 'let ASTAR_DIST = ~w~n~n', [DistA]),
    format(S, '# ASTAR_STEPS: per step [current, [candidate neighbours], chosen].~n', []),
    write_alg(S, 'ASTAR_STEPS', AstarSteps),
    format(S, '# ASTAR_PENDING: the full priority queue (open list) after each expansion, as~n', []),
    format(S, '# [node_id, f] pairs in ascending f order. The head is the node A* expands next.~n', []),
    write_alg(S, 'ASTAR_PENDING', AstarPending),
    format(S, '~n', []),

    write_alg(S, 'HILL_EXPAND', OrderH),
    write_alg(S, 'HILL_PATH', PathH),
    format(S, 'let HILL_DIST = ~w~n~n', [DistH]),
    format(S, '# HILL_STATUS is "reach" or "stuck" (Hill Climbing may dead-end).~n', []),
    format(S, 'let HILL_STATUS = "~w"~n~n', [StatusH]),
    format(S, '# HILL_STEPS: per step [current, [[candidate, edge_weight], ...], chosen];~n', []),
    format(S, '# candidates are in graph-edge order and chosen is the minimum-weight neighbour.~n', []),
    write_alg(S, 'HILL_STEPS', HillSteps),
    format(S, '~n', []),

    format(S, '# Best-First: greedy search on the heuristic h only (straight-line distance to the goal).~n', []),
    write_alg(S, 'BEST_EXPAND', OrderBest),
    write_alg(S, 'BEST_PATH', PathBest),
    format(S, 'let BEST_DIST = ~w~n~n', [DistBest]),
    format(S, '# BEST_STEPS: per step [current, [[candidate, h], ...], next first-visit node (-1 at the end)].~n', []),
    write_alg(S, 'BEST_STEPS', BestSteps),
    format(S, '# BEST_PENDING: open list after each expansion as [node_id, h] pairs in queue order.~n', []),
    write_alg(S, 'BEST_PENDING', BestPending),
    format(S, '# BEST_PRED: per step, the predecessor edge [parent, child] ([] for the start).~n', []),
    write_alg(S, 'BEST_PRED', BestPred),
    format(S, '~n', []),

    format(S, '# BFS / DFS: unweighted traversals over the same network. *_EXPAND is the~n', []),
    format(S, '# first-visit order; *_PATH is the path found from start to goal.~n', []),
    write_alg(S, 'BFS_EXPAND', OrderB),
    write_alg(S, 'BFS_PATH', PathB),
    format(S, '# BFS_STEPS: per step [current, queue_after, [[parent, child], ...], depth].~n', []),
    write_alg(S, 'BFS_STEPS', BfsSteps),
    write_alg(S, 'DFS_EXPAND', OrderDfs),
    write_alg(S, 'DFS_PATH', PathDfs),
    format(S, '# DFS_STEPS: per step [current, current_path, stack_heads,~n', []),
    format(S, '# [[parent, child], ...], depth, backtracked (1/0)].~n', []),
    write_alg(S, 'DFS_STEPS', DfsSteps),
    close(S).

write_alg(S, Name, List) :-
    format(S, 'let ~w = ', [Name]),
    emit_list(S, List),
    format(S, '~n', []).

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
