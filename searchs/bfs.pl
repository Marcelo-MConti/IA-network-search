:- ensure_loaded('dfs.pl').

bfs(Vi,Vf,Vertices,Arestas,Caminho):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    bfs_busca(Vf,Arestas,[Vi-[Vi]],Caminho,[Vi]).

% Caso em que chegamos ao vertice final
bfs_busca(Vf,_,[Vf-CaminhoAtual|_],Caminho,_) :-
    reverse(CaminhoAtual,Caminho), !.

% Caso em que não encontramos nenhum caminho possivel
bfs_busca(Vf,_,[],_Caminho,_):-
    fail.

bfs_busca(Vf,Arestas,[Atual-CaminhoAtual | DemaisCaminhos],Caminho,Fechados):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    % ignora nós já visitados (conjunto fechado) para manter busca limitada
    exclude(member_right(Fechados), L, Lfiltrado),
    adicionar_caminhos_bfs(Lfiltrado,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados),
    bfs_busca(Vf,Arestas,CaminhosAtualizados,Caminho,[Atual|Fechados]).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_bfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados):-
    findall(
        X-[X|CaminhoAtual],
        member(X,L),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,CaminhosAtualizados).