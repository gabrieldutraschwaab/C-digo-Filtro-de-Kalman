using UnityEngine;

public class TesteKalman : MonoBehaviour
{
    // Arraste as esferas aqui no Inspector da Unity
    public Transform esferaReal;
    public Transform esferaComRuido;
    public Transform esferaFiltrada;

    [Range(0f, 5f)]
    public float intensidadeDoRuido = 1.5f;

    // Configurações do movimento circular da Esfera Verde
    private float angulo = 0f;
    public float velocidade = 1.5f;
    public float raio = 4f;

    // Referência ao SEU script de filtro
    public FiltroKalmanGeolocalizacao seuFiltro;

    void Update()
    {
        // 1. MOVIMENTO REAL (Esfera Verde)
        angulo += velocidade * Time.deltaTime;
        float xReal = Mathf.Cos(angulo) * raio;
        float zReal = Mathf.Sin(angulo) * raio;
        Vector3 posicaoReal = new Vector3(xReal, 0f, zReal);
        
        if (esferaReal != null) esferaReal.position = posicaoReal;

        // 2. INJETAR RUÍDO BRUTO (Esfera Vermelha - Simulação do UWB Sujo)
        float ruidoX = Random.Range(-intensidadeDoRuido, intensidadeDoRuido);
        float ruidoZ = Random.Range(-intensidadeDoRuido, intensidadeDoRuido);
        Vector3 posicaoComRuido = new Vector3(xReal + ruidoX, 0f, zReal + ruidoZ);
        
        if (esferaComRuido != null) esferaComRuido.position = posicaoComRuido;

        // 3. PASSAR O DADO SUJO PARA O SEU FILTRO PROCESSAR
        if (seuFiltro != null)
        {
            // Enviamos a posição suja para o método que criaste
            seuFiltro.ReceberMedicaoUWB(posicaoComRuido);
            
            // A Esfera Azul (Filtrada) assume a posição que o teu filtro calculou
            if (esferaFiltrada != null)
            {
                esferaFiltrada.position = seuFiltro.transform.position;
            }
        }
    }
}