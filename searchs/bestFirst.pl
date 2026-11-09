:- ensure_loaded('dfs.pl').

best_first(Vi,Vf,Vertices,Arestas,Caminho,Distancia):-
    member(v(Vi,_,_),Vertices),
    member(v(Vf,_,_),Vertices),
    best_first_busca(Vf,Vertices,Arestas,[0-(0-(Vi-[Vi]))],Caminho,Distancia).

best_first_busca(Vf,_,_,[_-(Dist-(Vf-CaminhoAtual))|_], Caminho,Distancia):-
    reverse(CaminhoAtual,Caminho),
    Distancia=Dist.

best_first_busca(Vf,Vertices,Arestas,[(H-(Dist-(Atual-CaminhoAtual)))| DemaisCaminhos],Caminho,Distancia):-
    filhos(Atual,Arestas,CaminhoAtual,L),
    adicionar_caminhos_best(Vf,Vertices,L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Atual,Dist),
    best_first_busca(Vf,Vertices,Arestas,CaminhosAtualizados,Caminho,Distancia).

adicionar_caminhos_best(Vf,Vertices,L,CaminhoAtual,DemaisCaminhos,CaminhosAtualizados,Atual,Dist):-
    findall(
        D-(NovaDist-(X-[X|CaminhoAtual])),
        (
            member(X,L),
            member(v(X,X1,Y1),Vertices),
            member(v(Vf,X2,Y2),Vertices),
            member(v(Atual,AtualX,AtualY),Vertices),
            distancia(v(X,X1,Y1),v(Vf,X2,Y2),D),
            distancia(v(Atual,AtualX,AtualY),v(X,X1,Y1),Add),
            NovaDist is Dist + Add
        ),
        CaminhosNovos
    ),
    append(DemaisCaminhos,CaminhosNovos,S),
    keysort(S,CaminhosAtualizados).
