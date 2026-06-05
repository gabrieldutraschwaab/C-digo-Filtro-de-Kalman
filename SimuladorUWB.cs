// ============================================================
// SimuladorUWB.cs
// Simulador de medições de rádio UWB com ruído gaussiano
// ============================================================

using UnityEngine;

public class SimuladorUWB : MonoBehaviour
{
    [Header("Conexões do Sistema")]
    public FiltroKalmanGeolocalizacao filtroKalman; 
    public Transform operadorAlvo; 

    [Header("Configurações do Sensor UWB")]
    [Range(1f, 50f)]
    public float frequenciaAtualizacao = 10f; 
    public float erroDesvioPadrao = 0.3f; 

    [Header("Simulação de Obstáculos (NLOS)")]
    public bool simularInterferenciaNLOS = false;
    [Range(0f, 1f)]
    public float chanceDeOutlier = 0.05f; 

    private float cronometro = 0f;

    void Update()
    {
        if (filtroKalman == null || operadorAlvo == null) return;

        cronometro += Time.deltaTime;
        float intervalo = 1f / frequenciaAtualizacao;

        if (cronometro >= intervalo)
        {
            cronometro = 0f;
            GerarEMandarMedicao();
        }
    }

    void GerarEMandarMedicao()
    {
        Vector3 posicaoReal = operadorAlvo.position;

        float ruidoX = GerarRuidoGaussiano(0f, erroDesvioPadrao);
        float ruidoY = GerarRuidoGaussiano(0f, erroDesvioPadrao * 0.5f); 
        float ruidoZ = GerarRuidoGaussiano(0f, erroDesvioPadrao);

        Vector3 posicaoComRuido = posicaoReal + new Vector3(ruidoX, ruidoY, ruidoZ);

        if (simularInterferenciaNLOS && Random.value < chanceDeOutlier)
        {
            posicaoComRuido += Random.insideUnitSphere * Random.Range(3f, 7f);
        }

        filtroKalman.ReceberMedicaoUWB(posicaoComRuido);
    }

    float GerarRuidoGaussiano(float media, float desvioPadrao)
    {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return media + desvioPadrao * randStdNormal;
    }

    void OnDrawGizmos()
    {
        if (operadorAlvo != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, operadorAlvo.position);
        }
    }
}
