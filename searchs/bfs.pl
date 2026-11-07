:- include('dfs.pl').

bfs(Vi,Vf,Vertices,Arestas,Caminho):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    bfs_busca(Vf,Arestas,[Vi-[Vi]],Caminho).

% Caso em que chegamos ao vertice final
bfs_busca(Vf,_,[Vf-CaminhoAtual|Resto],Caminho) :-
    reverse(CaminhoAtual,Caminho).

% Caso em que não encontramos nenhum caminho possivel
bfs_busca(Vf,_,[],Caminho):-
    fail.

bfs_busca(Vf,Arestas,[Atual-CaminhoAtual | DemaisCaminhos],Caminho):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    adicionar_caminhos_bfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados),
    bfs_busca(Vf,Arestas,CaminhosAtualizados,Caminho).

% Responsável por adicionar os novos caminhos dos filhos
adicionar_caminhos_bfs(L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados):-
    findall(
        X-[X|CaminhoAtual],
        member(X,L),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,CaminhosAtualizados).