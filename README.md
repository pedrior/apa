# Projeto da disciplina de Análise e Projeto de Algoritmos

## Problema

“Seja ```G = (V, A)``` um grafo orientado onde, ```V = {0, 1, . . . , n}``` é o conjunto de vértices, o vértice 0 representa o depósito, ```n``` é o número de clientes a serem atendidos, e ```A``` é o conjunto de arcos do grafo. Um arco ```(i, j) ∈ A``` representa o caminho que um veículo deve percorrer para ir do ponto ```i``` ao ```j```, e o custo associado a tal arco é dado por ```cij``` . Cada cliente ```i``` possui uma demanda ```di``` de pacotes a serem entregues. A empresa possui uma frota com ```k``` veículos,
todos do mesmo modelo e com capacidade ```Q```. Todos os veículos devem iniciar e terminar suas rotas no depósito, a capacidade máxima deve ser respeitada e cada veículo utilizado incorre em um custo ```r``` para a empresa. Visto que existe a opção de terceirizar entregas, caso um cliente ```i``` não seja atendido por nenhum veículo, deve ser pago um valor ```pi``` para que a entrega seja realizada por outra empresa. A fim de garantir uma utilização mínima de sua frota e de seus funcionários, a empresa estabelece que ao menos ```L``` entregas devem ser realizas sem terceirização. O objetivo do problema é encontrar o conjunto de rotas que minimize a soma do custo de roteamento (custo dos arcos), do custo associado à utilização dos veículos e do total pago com a terceirização de entregas.”

## Requisitos

- [x] Implementação de ao menos uma heurística de construção (algoritmo guloso) para a geração de uma solução viável.

- [x] Implementação de estruturas de vizinhança.
  - [x] Com movimentos envolvendo uma única rota.
  - [x] Com movimentos envolvendo múltiplas rotas.
  - [x] Capaz de lidar com entregas terceirizadas
    
- [x] Implementação do algoritmo de busca local __VND__ (_Variable Neighborhood Descent_).
- [ ] Implementação de uma meta-heurística (_opcional_). Sugestões: __GRASP__ ou __ILS__.

## Entrada

A estrutura do arquivo contendo os dados de uma instância do problema a ser utilizada é o seguinte:

> n\
> k\
> Q\
> L\
> r\
>\
> array d\
>\
> array p\
>\
> matriz c

## Saída

A estrutura do arquivo de saída contento a melhor solução encontrada é a seguinte:

> \<valor total da solucao\>\
> \<custo de roteamento\>\
> \<custo associado a utilizacao dos veiculos\>\
> \<custo de terceirizacao\>
>
> \<lista de clientes terceirizados\>
>
> \<numero de rotas\>\
> \<rota 1\>\
> \<rota 2\>



## Compilando e executando

1. No diretório raiz do projeto, crie o diretório de sáida com o comando ```mkdir obj```
2. Compile utlizando o MinGW com o comando ```mingw32-make ```
3. Execute com um arquivo de entrada com o comando ```./bin/./apa <arquivo-de-entrada>```

## Autores:

- [Pedro Júnior](https://github.com/pedrior)
- Miguel Amádio
- Ryan Leal
