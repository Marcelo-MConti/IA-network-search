:- ensure_loaded('dfs.pl').

bfs(Vi,Vf,Vertices,Arestas,Caminho):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    bfs_busca(Vf,Arestas,[Vi-[Vi]],Caminho,[Vi],[Vi]).

% Caso em que chegamos ao vertice final
bfs_busca(Vf,_,[Vf-CaminhoAtual|_],Caminho,_,_) :-
    reverse(CaminhoAtual,Caminho), !.

% Caso em que não encontramos nenhum caminho possivel
bfs_busca(Vf,_,[],_Caminho,_,_):-
    fail.

bfs_busca(Vf,Arestas,[Atual-CaminhoAtual | DemaisCaminhos],Caminho,Fechados,Visitados):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já expandidos ou já enfileirados (evita duplicatas na fila)
    exclude(member_right(Fechados), L, L1),
    exclude(member_right(Visitados), L1, Lfiltrado),
    adicionar_caminhos_bfs(Lfiltrado,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados),
    % registra os novos enfileirados para não reenfileirá-los depois
    findall(X, member(X, Lfiltrado), NovosVisitados),
    append(NovosVisitados, Visitados, VisitadosNovo),
    bfs_busca(Vf,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados],VisitadosNovo).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_bfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados):-
    findall(
        X-[X|CaminhoAtual],
        member(X,L),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,CaminhosAtualizados).