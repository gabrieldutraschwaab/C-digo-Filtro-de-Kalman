// ============================================================
// SimuladorUWB.cs
// Simula leituras de um módulo UWB via BLE para testes no editor
// Substitua por SimuladorUWBBluetooth.cs quando tiver hardware real
// ============================================================

using UnityEngine; // Importa as ferramentas básicas da Unity (vetores, física, logs)

public class SimuladorUWB : MonoBehaviour // Herda de MonoBehaviour, ou seja, pode ser colado num objeto do cenário
{
    [Header("Referência ao EKF")] // Cria um cabeçalho visual no painel Inspector da Unity
    public FiltroKalmanGeolocalizacao ekfScript; // Campo para arrastares o objeto que tem o script do Kalman colado

    [Header("Parâmetros da Simulação")] // Cabeçalho para agrupar as configurações de teste
    [Tooltip("Frequência de envio de dados UWB (Hz). UWB real: 10-100 Hz.")] // Texto de ajuda que aparece ao passar o rato por cima
    [Range(1f, 100f)] // Cria uma barra deslizante no Inspector que limita os valores entre 1 e 100
    public float frequenciaUWB = 10f; // Define quantas vezes por segundo o rádio simulado vai enviar a posição (padrão: 10Hz)

    [Tooltip("Erro máximo simulado do UWB em LOS (metros). Típico: 0.1-0.3m")]
    [Range(0.05f, 0.5f)] // Limita o erro normal em linha de vista entre 5 centímetros e 50 centímetros
    public float erroLOS = 0.15f; // Margem de erro padrão do UWB em condições perfeitas (15 centímetros)

    [Tooltip("Simular evento de NLOS (leitura corrompida)?")]
    public bool simularNLOS = false; // Caixa de seleção (ligar/desligar) para simular interferências de metal ou paredes

    [Tooltip("Magnitude do erro de NLOS (metros). Típico: 1-3m")]
    [Range(0.5f, 5.0f)] // Limita o erro de ricochete entre meio metro e 5 metros
    public float erroNLOS = 2.0f; // Tamanho do salto bizarro que a posição vai dar quando o sinal bater numa máquina (2 metros)

    [Tooltip("Probabilidade de ocorrência de NLOS por ciclo (0-1).")]
    [Range(0f, 1f)] // Limita a probabilidade entre 0% e 100%
    public float probabilidadeNLOS = 0.1f; // Chance de ocorrer um erro NLOS a cada envio (0.1 significa 10% de chance)

    private float temporizador = 0f; // Variável interna para contar o tempo que passou entre cada envio
    private float intervalo; // Guarda o tempo exato de espera em segundos entre um envio e outro (ex: 1/10Hz = 0.1s)

    void Update() // Executado automaticamente a cada frame da Unity
    {
        if (ekfScript == null) return; // Se não arrastaste o script do Kalman para o campo, para o código aqui para não dar erro

        intervalo = 1f / frequenciaUWB; // Calcula o tempo de espera (se a frequência é 10Hz, o intervalo é 0.1 segundos)
        temporizador += Time.deltaTime; // Soma o tempo que o frame atual demorou a rodar ao cronómetro

        if (temporizador >= intervalo) // Se o cronómetro atingiu o tempo do intervalo (ex: passaram-se 0.1 segundos)...
        {
            EnviarMedicaoSimulada(); // Chama a função que fabrica a leitura do UWB com ruído
            temporizador = 0f; // Zera o cronómetro para recomeçar a contagem do próximo ciclo
        }
    }

    void EnviarMedicaoSimulada() // Função que calcula e envia os dados falsos de UWB
    {
        // Posição real atual do objeto (Usa a posição do próprio avatar onde este script está colado na Unity)
        Vector3 posicaoReal = transform.position; 

        // CRITÉRIO DE DECISÃO: Decide se este envio específico vai sofrer interferência (NLOS) ou não
        // Random.value gera um número aleatório entre 0.0 e 1.0. Se for menor que 0.1 (10%), ativa o erro
        bool eNLOS = simularNLOS && (Random.value < probabilidadeNLOS); 

        // Se for um evento NLOS, a força do erro será de 2.0m (erroNLOS), caso contrário será de 0.15m (erroLOS)
        float erro = eNLOS ? erroNLOS : erroLOS; 

        // ADICIONA RUÍDO GAUSSIANO REAL: Usa o algoritmo de Box-Muller para criar erros em forma de curva de sino
        float noiseX = GaussianNoise(0, erro); // Gera o ruído aleatório gaussiano para o eixo X
        float noiseY = GaussianNoise(0, erro * 0.5f); // Gera ruído para o eixo Y (altura) cortado ao meio (eixo Y costuma falhar menos)
        float noiseZ = GaussianNoise(0, erro); // Gera o ruído aleatório gaussiano para o eixo Z

        // Fabrica a coordenada final do UWB somando a posição real do operador com o ruído gerado
        float uwbX = posicaoReal.x + noiseX; // Posição X barulhenta
        float uwbY = posicaoReal.y + noiseY; // Posição Y barulhenta
        float uwbZ = posicaoReal.z + noiseZ; // Posição Z barulhenta

        if (eNLOS) // Se este ciclo foi um erro de ricochete...
        {
            // Mostra um aviso amarelo no ecrã da Unity a avisar que um erro gigante foi injetado de propósito
            Debug.LogWarning($"[SimUWB] Evento NLOS simulado! Erro={erro:F2}m"); 
        }

        // CÁBULA PARA A DEFESA: Repara que o teu código original tentava passar 3 floats isolados: ekfScript.ReceberMedicaoUWB(uwbX, uwbY, uwbZ);
        // Mas o teu script FiltroKalmanGeolocalizacao.cs espera receber um "Vector3". 
        // Abaixo corrigi a linha para empacotar os 3 números num Vector3 unificado, garantindo que a Unity compila sem erros!
        ekfScript.ReceberMedicaoUWB(new Vector3(uwbX, uwbY, uwbZ)); 
    }

    // =========================================================================
    // ALGORITMO DE BOX-MULLER: Transforma números aleatórios comuns da Unity
    // em Ru
