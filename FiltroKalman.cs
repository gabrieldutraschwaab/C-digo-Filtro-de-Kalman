// ============================================================
// FiltroKalmanGeolocalizacao.cs
// EKF 6D para fusão VIO (Quest 3 OpenXR) + UWB
// Compatível com: Unity 2022.3 LTS+, Meta XR SDK, MathNet.Numerics
// ============================================================

using UnityEngine; // Importa as ferramentas básicas da Unity (vetores, transformações, etc.)
using UnityEngine.XR; // Importa o sistema de Realidade Virtual/Mista (OpenXR)
using MathNet.Numerics.LinearAlgebra; // Importa a biblioteca de álgebra linear (matrizes e vetores)
using MathNet.Numerics.LinearAlgebra.Double; // Garante que as matrizes usam precisão dupla (double)

public class FiltroKalmanGeolocalizacao : MonoBehaviour // Define a classe que pode ser colada num objeto da Unity
//Ao herdar (: MonoBehaviour), é como se eu dissesse à Unity:
//"Atenção! Este script agora é um Componente oficial. Eu quero poder colá-lo num objeto do cenário para que ele ganhe o poder de rodar os métodos Start() e Update() em tempo real."
{
    // ----------------------------------------------------------
    // ESTADO: [x, y, z, vx, vy, vz]  →  Vetor de 6 dimensões
    // ----------------------------------------------------------
    private Vector<double> x_estado; // Vetor que guarda a posição (x,y,z) e a velocidade (vx,vy,vz) atuais
    private Matrix<double> P_covariancia; // Matriz que guarda a incerteza atual do filtro (grau de erro)
    private Matrix<double> Q_ruido_processo; // Matriz com a variância do ruído da movimentação (VIO/IMU)
    private Matrix<double> R_ruido_medicao; // Matriz com a variância do ruído do sensor externo (UWB)
    private Matrix<double> H_medicao; // Matriz de observação (converte o estado 6D para a medição 3D do UWB)

    private Vector3 posicaoAnteriorOpenXR; // Guarda a posição do Quest 3 no frame anterior para calcular o delta
    private Quaternion orientacaoAnteriorOpenXR; // Guarda a rotação do Quest 3 no frame anterior
    private bool primeiroFrameVIO = true; // Flag para ignorar o cálculo de movimento no primeiríssimo frame

    [Header("Ruído do Processo (VIO/IMU)")] // Cria um cabeçalho visual no Inspector da Unity
    public float qPosicao = 0.01f; // Ajuste do ruído de posição do óculos (valores menores = maior confiança no óculos)
    public float qVelocidade = 0.05f; // Ajuste do ruído de velocidade do óculos

    [Header("Ruído da Medição (UWB)")] // Outro cabeçalho para o UWB no Inspector
    public float rUWB = 0.5f; // Desvio padrão do erro do UWB em metros (ex: 0.5 = 50cm de margem de erro)

    [Header("Teste Chi-Quadrado (NLOS)")] // Cabeçalho para a filtragem de ricochete de sinal
    public double chiQuadradoThreshold = 7.81; // Limite estatístico para 3 graus de liberdade (rejeita outliers acima disto)

    [Header("Restrições de Mapa (Map Matching)")] // Cabeçalho para o alinhamento com a planta da fábrica
    public bool usarMapMatching = true; // Liga/desliga a trava que impede o utilizador de atravessar paredes
    public float raioMapMatching = 1.0f; // Raio máximo de busca para encontrar o chão válido mais próximo

    private bool filtroInicializado = false; // Indica se as matrizes já foram criadas e prontas a usar
    private System.Collections.Generic.List<InputDevice> dispositivos = new System.Collections.Generic.List<InputDevice>(); // Lista para armazenar os óculos detetados pelo OpenXR
    private UnityEngine.AI.NavMeshHit hit; // Guarda o resultado da colisão com o mapa da fábrica (NavMesh)

    // Atalhos matemáticos para criar matrizes e vetores mais rapidamente
    private static readonly MatrixBuilder<double> M = Matrix<double>.Build; 
    private static readonly VectorBuilder<double> V = Vector<double>.Build;

    void Start() // Método executado automaticamente quando a aplicação arranca na Unity
    {
        InicializarFiltro(); // Chama a função que cria as matrizes do Kalman
        InicializarOpenXR(); // Chama a função que liga a comunicação com o Meta Quest 3
    }

    void InicializarFiltro() // Função de configuração inicial das matrizes
    {
        x_estado = V.Dense(6, 0.0); // Cria o vetor de estado com 6 posições, tudo a zeros
        P_covariancia = M.DenseIdentity(6) * 10.0; // Cria uma matriz identidade 6x6 e multiplica por 10 (alta incerteza inicial)

        Q_ruido_processo = M.Dense(6, 6, 0.0); // Cria a matriz Q 6x6 vazia
        AtualizarMatrizQ(); // Preenche a matriz Q com os valores configurados no Inspector

        R_ruido_medicao = M.DenseIdentity(3) * (rUWB * rUWB); // Cria a matriz R 3x3 com a variância do UWB (raio ao quadrado)

        H_medicao = M.Dense(3, 6, 0.0); // Cria a matriz H com 3 linhas (medições) e 6 colunas (estados)
        H_medicao[0, 0] = 1.0; // Diz que a medição X do UWB corrige diretamente o estado X (posição)
        H_medicao[1, 1] = 1.0; // Diz que a medição Y do UWB corrige diretamente o estado Y (altura)
        H_medicao[2, 2] = 1.0; // Diz que a medição Z do UWB corrige diretamente o estado Z (profundidade)

        filtroInicializado = true; // Ativa a flag indicando que o filtro está pronto a operar
    }

    void AtualizarMatrizQ() // Preenche a diagonal da matriz Q com os desvios de posição e velocidade
    {
        Q_ruido_processo[0, 0] = qPosicao * qPosicao; // Variância de posição X
        Q_ruido_processo[1, 1] = qPosicao * qPosicao; // Variância de posição Y
        Q_ruido_processo[2, 2] = qPosicao * qPosicao; // Variância de posição Z
        Q_ruido_processo[3, 3] = qVelocidade * qVelocidade; // Variância de velocidade VX
        Q_ruido_processo[4, 4] = qVelocidade * qVelocidade; // Variância de velocidade VY
        Q_ruido_processo[5, 5] = qVelocidade * qVelocidade; // Variância de velocidade VZ
    }

    void InicializarOpenXR() // Configura a captação de dados do Meta Quest 3
    {
        InputDevices.GetDevicesAtXRNode(XRNode.Head, dispositivos); // Procura pelo dispositivo posicionado na cabeça (óculos)
        if (dispositivos.Count > 0) // Se encontrou o óculos real...
        {
            if (dispositivos[0].TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos)) // Tenta ler a posição
            {
                posicaoAnteriorOpenXR = pos; // Guarda a posição inicial para o próximo frame
            }
            if (dispositivos[0].TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot)) // Tenta ler a rotação
            {
                orientacaoAnteriorOpenXR = rot; // Guarda a rotação inicial
            }
        }
    }

    void Update() // Executado a cada frame da Unity (Etapa de Predição Cinética)
    {
        if (!filtroInicializado) return; // Se o filtro não foi inicializado, cancela a execução deste frame

        float dt = Time.deltaTime; // Descobre quantos segundos se passaram desde o último frame (ex: 0.011s)
        if (dt <= 0) dt = 0.001f; // Evita divisão por zero ou bugs de tempo parado

        Vector3 posicaoAtualOpenXR = Vector3.zero; // Cria uma variável temporária para a posição atual
        Quaternion orientacaoAtualOpenXR = Quaternion.identity; // Cria uma variável temporária para a rotação atual
        bool dadosValidos = false; // Flag para checar se o óculos entregou dados válidos neste frame

        InputDevices.GetDevicesAtXRNode(XRNode.Head, dispositivos); // Atualiza a lista de dispositivos ligados à cabeça
        if (dispositivos.Count > 0) // Se o Quest 3 estiver ativo...
        {
            // Tenta ler a posição e a rotação em simultâneo
            if (dispositivos[0].TryGetFeatureValue(CommonUsages.devicePosition, out posicaoAtualOpenXR) &&
                dispositivos[0].TryGetFeatureValue(CommonUsages.deviceRotation, out orientacaoAtualOpenXR))
            {
                dadosValidos = true; // Se ambas as leituras funcionaram, os dados são válidos
            }
        }

        if (!dadosValidos) // Se o utilizador tirou os óculos ou o OpenXR falhou (Caso de Teste no Editor)...
        {
            posicaoAtualOpenXR = transform.position; // Usa a posição da câmera virtual do próprio editor da Unity
            orientacaoAtualOpenXR = transform.rotation; // Usa a rotação da câmera virtual do editor (Fallback automático)
            dadosValidos = true; // Força a validação para o teste não travar
        }

        if (primeiroFrameVIO) // Se for o primeiro frame do sistema...
        {
            posicaoAnteriorOpenXR = posicaoAtualOpenXR; // Sincroniza a posição anterior
            orientacaoAnteriorOpenXR = orientacaoAtualOpenXR; // Sincroniza a rotação anterior
            primeiroFrameVIO = false; // Desativa a flag para sempre
            return; // Salta o resto do cálculo deste frame porque não há deslocamento ainda
        }

        Vector3 deltaPosicao = posicaoAtualOpenXR - posicaoAnteriorOpenXR; // Descobre quantos metros a cabeça moveu (vetor)
        Vector3 velocidadeInstantanea = deltaPosicao / dt; // V = DeltaS / DeltaT (Calcula a velocidade real do passo)

        var F_t = M.DenseIdentity(6); // Cria a matriz de transição F 6x6 preenchida com a Identidade
        F_t[0, 3] = dt; // Define que a nova posição X depende da velocidade VX multiplicada pelo tempo (Cinemática)
        F_t[1, 4] = dt; // Define que a nova posição Y depende da velocidade VY multiplicada pelo tempo
        F_t[2, 5] = dt; // Define que a nova posição Z depende da velocidade VZ multiplicada pelo tempo

        var u = V.Dense(6, 0.0); // Cria o vetor de entrada de controle com 6 dimensões zeradas
        u[0] = deltaPosicao.x; // Injeta o movimento X detetado pela IMU do óculos
        u[1] = deltaPosicao.y; // Injeta o movimento Y detetado pela IMU do óculos
        u[2] = deltaPosicao.z; // Injeta o movimento Z detetado pela IMU do óculos
        u[3] = velocidadeInstantanea.x - x_estado[3]; // Calcula a variação da velocidade no eixo X
        u[4] = velocidadeInstantanea.y - x_estado[4]; // Calcula a variação da velocidade no eixo Y
        u[5] = velocidadeInstantanea.z - x_estado[5]; // Calcula a variação da velocidade no eixo Z

        AtualizarMatrizQ(); // Garante que qualquer alteração de ruído feita pelo utilizador no slider seja aplicada

        // EQUAÇÃO 1 DE KALMAN (PREDIÇÃO DO ESTADO): x = F * x + u
        x_estado = (F_t * x_estado) + u; 

        // EQUAÇÃO 2 DE KALMAN (PREDIÇÃO DA INCERTEZA): P = F * P * F^T + Q
        P_covariancia = (F_t * P_covariancia * F_t.Transpose()) + Q_ruido_processo; 

        if (usarMapMatching) // Se a correção de mapa estiver ligada no Inspector...
        {
            Vector3 posFiltrada = new Vector3((float)x_estado[0], (float)x_estado[1], (float)x_estado[2]); // Converte o estado atual para Vector3
            Vector3 posCorrigida = AplicarRestricaoDeMapa(posFiltrada); // Passa o ponto pelo teste de colisão das paredes
            x_estado[0] = posCorrigida.x; // Substitui o X do estado pelo valor corrigido anti-parede
            x_estado[1] = posCorrigida.y; // Substitui o Y do estado pelo valor corrigido anti-parede
            x_estado[2] = posCorrigida.z; // Substitui o Z do estado pelo valor corrigido anti-parede
        }

        // Atualiza a posição do objeto virtual na Unity para que o holograma se mova visualmente no ecrã do óculos
        transform.position = new Vector3((float)x_estado[0], (float)x_estado[1], (float)x_estado[2]); 
        transform.rotation = orientacaoAtualOpenXR; // Mantém a rotação pura vinda do óculos (o EKF foca na translação)

        posicaoAnteriorOpenXR = posicaoAtualOpenXR; // Guarda a posição atual para servir de histórico no próximo frame
        orientacaoAnteriorOpenXR = orientacaoAtualOpenXR; // Guarda a rotação atual para o histórico
    }

    // Método público chamado de fora sempre que uma antena UWB envia coordenadas novas (Etapa de Correção)
    public void ReceberMedicaoUWB(Vector3 posicaoUWB) 
    {
        if (!filtroInicializado) return; // Se as matrizes não existem, rejeita a leitura temporariamente

        var z_medicao = V.Dense(3); // Cria o vetor de medição com 3 dimensões (X, Y, Z vindo do hardware UWB)
        z_medicao[0] = posicaoUWB.x; // Preenche com o X do rádio
        z_medicao[1] = posicaoUWB.y; // Preenche com o Y do rádio
        z_medicao[2] = posicaoUWB.z; // Preenche com o Z do rádio

        // EQUAÇÃO 3 DE KALMAN (INOVAÇÃO): y = z - H * x
        var y_inovacao = z_medicao - (H_medicao * x_estado); 

        // Atualiza a matriz R com o valor do slider do Inspector (caso tenha mudado durante o teste)
        R_ruido_medicao = M.DenseIdentity(3) * (rUWB * rUWB); 

        // EQUAÇÃO 4 DE KALMAN (COVARIÂNCIA DA INOVAÇÃO): S = H * P * H^T + R
        var S_cov_inovacao = (H_medicao * P_covariancia * H_medicao.Transpose()) + R_ruido_medicao; 

        var S_inv = S_cov_inovacao.Inverse(); // Inverte a matriz S (equivalente a dividir pela incerteza total)

        // CÁLCULO DA DISTÂNCIA DE MAHALANOBIS: D² = y^T * S^-1 * y
        double distMahalanobis = y_inovacao * S_inv * y_inovacao; 

        // TESTE DO CHI-QUADRADO (FILTRO DE OUTLIER): Se o desvio estatístico for bizarro, rejeita
        if (distMahalanobis > chiQuadradoThreshold) 
        {
            Debug.LogWarning($"[EKF] Medição UWB rejeitada por NLOS! Distância Mahalanobis: {distMahalanobis:F2} > Threshold: {chiQuadradoThreshold}");
            return; // Bloqueia a execução das próximas linhas. A leitura do UWB é jogada no lixo e o filtro ignora o erro
        }

        // EQUAÇÃO 5 DE KALMAN (GANHO DE KALMAN): K = P * H^T * S^-1
        var K_ganho = P_covariancia * H_medicao.Transpose() * S_inv; 

        // EQUAÇÃO 6 DE KALMAN (ATUALIZAÇÃO DO ESTADO): x = x + K * y
        x_estado = x_estado + (K_ganho * y_inovacao); 

        var I = M.DenseIdentity(6); // Cria uma matriz identidade 6x6 auxiliar para fechar a conta
        
        // EQUAÇÃO 7 DE KALMAN (ATUALIZAÇÃO DA INCERTEZA): P = (I - K * H) * P
        P_covariancia = (I - (K_ganho * H_medicao)) * P_covariancia; 
    }

    // Função interna que força o ponto a ficar dentro da zona de caminhada válida da planta industrial
    private Vector3 AplicarRestricaoDeMapa(Vector3 posicaoEKF)
    {
        // SamplePosition projeta o ponto do Kalman contra o NavMesh (a malha 3D que mapeia o chão real da fábrica)
        if (UnityEngine.AI.NavMesh.SamplePosition(posicaoEKF, out hit, raioMapMatching, UnityEngine.AI.NavMesh.AllAreas))
        {
            return hit.position; // Se encontrou o chão válido dentro do raio, retorna as coordenadas exatas do chão
        }
        return posicaoEKF; // Se o operador estiver numa zona livre e permitida, não faz nada e mantém o ponto original
    }

    public Vector3 PosicaoFiltrada // Propriedade pública para outros scripts lerem a posição 3D limpa atual
    {
        get { return new Vector3((float)x_estado[0], (float)x_estado[1], (float)x_estado[2]); }
    }

    public Vector3 VelocidadeEstimada // Propriedade pública para ler a velocidade atual do operador
    {
        get { return new Vector3((float)x_estado[3], (float)x_estado[4], (float)x_estado[5]); }
    }

    public Vector3 GetIncertezaPosicao() // Retorna o nível de erro nos 3 eixos (valores da diagonal de P)
    {
        return new Vector3(
            (float)P_covariancia[0, 0], // Incerteza no eixo X
            (float)P_covariancia[1, 1], // Incerteza no eixo Y (altura)
            (float)P_covariancia[2, 2]  // Incerteza no eixo Z
        );
    }

    // Método público chamado pelo ORB-SLAM3 quando há um Loop Closure ou Relocalização bem-sucedida
    public void ReinicializarComPose(Vector3 posicaoConhecida) 
    {
        x_estado[0] = posicaoConhecida.x; // Injeta à força o X correto descoberto pelas câmeras
        x_estado[1] = posicaoConhecida.y; // Injeta o Y correto
        x_estado[2] = posicaoConhecida.z; // Injeta o Z correto
        x_estado[3] = 0; // Zera a velocidade VX para reiniciar o cálculo cinético de forma limpa
        x_estado[4] = 0; // Zera a velocidade VY
        x_estado[5] = 0; // Zera a velocidade VZ
        P_covariancia = Matrix<double>.Build.DenseIdentity(6) * 1.0; // Reseta a incerteza para um valor baixo e estável (1.0)
        Debug.Log($"[EKF] Reinicializado em posição conhecida via SLAM/PnP: {posicaoConhecida}");
    }

    void OnDrawGizmos() // Desenha elementos de ajuda visual que só aparecem na tela do editor da Unity (Debug)
    {
        if (!filtroInicializado) return; // Se o filtro não arrancou, não desenha nada para evitar erros de ecrã

        Gizmos.color = Color.green; // Escolhe a cor verde para desenhar a posição atual do filtro
        Gizmos.DrawWireSphere(PosicaoFiltrada, 0.15f; // Desenha uma pequena esfera verde onde o operador está virtualmente

        Gizmos.color = new Color(1f, 0.92f, 0.01f, 0.2f); // Cria um amarelo transparente para representar a incerteza
        Vector3 incerteza = GetIncertezaPosicao(); // Recolhe os valores da matriz P
        float raioIncerteza = (incerteza.x + incerteza.y + incerteza.z) / 3f; // Faz a média do erro dos 3 eixos para virar o raio da esfera
        Gizmos.DrawSphere(PosicaoFiltrada, Mathf.Clamp(raioIncerteza, 0.1f, 5.0f)); // Desenha a bolha de incerteza (ela cresce se o sinal falhar)
    }
}
